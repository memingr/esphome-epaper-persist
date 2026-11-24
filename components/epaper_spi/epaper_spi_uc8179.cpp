#include "epaper_spi_uc8179.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_attr.h>
#include <nvs.h>
#endif

namespace esphome::epaper_spi {

ESPHOME_LOG_TAG(TAG, "epaper_spi.uc8179");

// ---------------------------------------------------------------------------
// Previous-frame persistence (local patch)
// ---------------------------------------------------------------------------
static constexpr uint32_t FRAME_MAGIC = 0x45504631;  // "EPF1"
static constexpr const char *NVS_NAMESPACE = "epd_frame";
static constexpr const char *NVS_KEY = "frame";

struct FrameHeader {
  uint32_t magic;
  uint16_t width;
  uint16_t height;
  uint32_t raw_length;
  uint32_t hash;
};

#ifdef USE_ESP32
// Zeroed on power-up, kept across deep sleep
RTC_DATA_ATTR static uint32_t rtc_magic;
RTC_DATA_ATTR static uint8_t rtc_update_count;
#endif

static uint32_t frame_hash(const std::vector<uint8_t> &data) {
  uint32_t hash = FNV1_OFFSET_BASIS;
  for (uint8_t b : data)
    hash = fnv1a_hash_extend(hash, b);
  return hash;
}

// Run-length encoding: ctrl < 0x80 -> ctrl+1 literal bytes follow;
// ctrl >= 0x80 -> the next byte repeated (ctrl & 0x7F) + 3 times.
static void rle_encode(const std::vector<uint8_t> &in, std::vector<uint8_t> &out) {
  const size_t n = in.size();
  size_t i = 0;
  while (i < n) {
    size_t run = 1;
    while (i + run < n && run < 130 && in[i + run] == in[i])
      run++;
    if (run >= 3) {
      out.push_back(static_cast<uint8_t>(0x80 | (run - 3)));
      out.push_back(in[i]);
      i += run;
      continue;
    }
    const size_t start = i;
    size_t count = 0;
    while (i < n && count < 128) {
      if (i + 2 < n && in[i] == in[i + 1] && in[i] == in[i + 2])
        break;  // a run starts here
      i++;
      count++;
    }
    out.push_back(static_cast<uint8_t>(count - 1));
    out.insert(out.end(), in.begin() + start, in.begin() + start + count);
  }
}

static bool rle_decode(const uint8_t *p, const uint8_t *end, std::vector<uint8_t> &out, size_t length) {
  out.assign(length, 0xFF);
  size_t o = 0;
  while (p < end && o < length) {
    const uint8_t ctrl = *p++;
    if (ctrl & 0x80) {
      const size_t count = (ctrl & 0x7F) + 3;
      if (p >= end || o + count > length)
        return false;
      std::memset(out.data() + o, *p++, count);
      o += count;
    } else {
      const size_t count = ctrl + 1;
      if (p + count > end || o + count > length)
        return false;
      std::memcpy(out.data() + o, p, count);
      p += count;
      o += count;
    }
  }
  return o == length && p == end;
}

bool EPaperUC8179::load_frame_() {
#ifdef USE_ESP32
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    return false;
  size_t size = 0;
  bool ok = false;
  if (nvs_get_blob(handle, NVS_KEY, nullptr, &size) == ESP_OK && size > sizeof(FrameHeader)) {
    std::vector<uint8_t> blob(size);
    if (nvs_get_blob(handle, NVS_KEY, blob.data(), &size) == ESP_OK) {
      FrameHeader hdr;
      std::memcpy(&hdr, blob.data(), sizeof(hdr));
      if (hdr.magic == FRAME_MAGIC && hdr.width == this->width_ && hdr.height == this->height_ &&
          hdr.raw_length == this->buffer_length_ &&
          rle_decode(blob.data() + sizeof(hdr), blob.data() + size, this->prev_, hdr.raw_length) &&
          frame_hash(this->prev_) == hdr.hash) {
        ESP_LOGD(TAG, "Restored previous frame from NVS (%u bytes compressed)", (unsigned) size);
        ok = true;
      }
    }
  }
  nvs_close(handle);
  if (!ok)
    this->prev_.clear();
  return ok;
#else
  return false;
#endif
}

void EPaperUC8179::store_frame_() {
  // Copy the frame now on the panel
  this->prev_.resize(this->buffer_length_);
  size_t index = 0;
  while (index != this->buffer_.size()) {
    size_t length;
    const uint8_t *data = this->buffer_.get_span(index, length);
    std::memcpy(this->prev_.data() + index, data, length);
    index += length;
  }
#ifdef USE_ESP32
  rtc_magic = 0;
  std::vector<uint8_t> blob(sizeof(FrameHeader));
  blob.reserve(this->buffer_length_ / 3);
  FrameHeader hdr{FRAME_MAGIC, this->width_, this->height_, (uint32_t) this->buffer_length_, frame_hash(this->prev_)};
  std::memcpy(blob.data(), &hdr, sizeof(hdr));
  rle_encode(this->prev_, blob);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(handle, NVS_KEY, blob.data(), blob.size());
    if (err == ESP_OK)
      err = nvs_commit(handle);
    nvs_close(handle);
  }
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Could not store frame in NVS: %s", esp_err_to_name(err));
    return;
  }
  rtc_update_count = this->update_count_;
  rtc_magic = FRAME_MAGIC;
  ESP_LOGD(TAG, "Stored frame in NVS (%u bytes), next update #%u of %u", (unsigned) blob.size(),
           (unsigned) this->update_count_, (unsigned) this->full_update_every_);
#endif
}

void EPaperUC8179::setup() {
  EPaperUltraChip::setup();
  if (this->is_failed() || !this->is_using_partial_update_())
    return;
#ifdef USE_ESP32
  if (rtc_magic == FRAME_MAGIC && rtc_update_count != 0 && this->load_frame_()) {
    this->update_count_ = rtc_update_count % this->full_update_every_;
    ESP_LOGI(TAG, "Woke with a known frame on the panel; next update #%u will be partial",
             (unsigned) this->update_count_);
  } else {
    this->update_count_ = 0;
    ESP_LOGI(TAG, "No known frame on the panel; next update will be full");
  }
#endif
}

void EPaperUC8179::deep_sleep() {
  // Called once at the end of every completed update
  if (this->is_using_partial_update_())
    this->store_frame_();
  EPaperUltraChip::deep_sleep();
}

// ---------------------------------------------------------------------------
// Driver (upstream, with the previous-frame plane added to partial updates)
// ---------------------------------------------------------------------------

bool EPaperUC8179::initialise(bool partial) {
  EPaperBase::initialise(partial);  // send the model init sequence
  ESP_LOGV(TAG, "Power on");
  this->command(CMD_POWER_ON);
  this->next_delay_ = 100;
  return true;
}

// Set up the refresh mode. Must be called after power-on has completed.
void EPaperUC8179::set_refresh_mode_() {
  if (!this->is_using_partial_update_()) {
    return;  // plain full refresh uses the mode set by the init sequence
  }
  // Fast and partial refresh use flipped data polarity and a floating border
  this->cmd_data(CMD_CDI, {0xA9, 0x07});
  // Force the waveform via the temperature registers: 0x5A selects the fast
  // full-refresh waveform, 0x6E the partial-refresh waveform
  this->cmd_data(CMD_CCSET, {0x02});
  if (this->partial_) {
    this->cmd_data(CMD_TSSET, {0x6E});
    this->set_partial_window_(0, this->width_ - 1, 0, this->height_ - 1);
  } else {
    this->cmd_data(CMD_TSSET, {0x5A});
    this->command(CMD_PARTIAL_OUT);  // exit partial mode
  }
}

bool HOT EPaperUC8179::transfer_data() {
  const uint32_t start_time = millis();
  const size_t buffer_length = this->buffer_length_;
  if (this->current_data_index_ == 0) {
    this->set_refresh_mode_();
  }
  const bool partial_mode = this->is_using_partial_update_();
  // Partial refresh: the previous image is restored into DTM1, so it is correct even after
  // the controller lost its RAM (deep sleep / reset)
  const bool send_prev = partial_mode && this->partial_ && this->prev_.size() == buffer_length;
  // Fast full refresh sends the inverse of the new image as the previous image, so every pixel transitions
  const bool two_pass = (partial_mode && !this->partial_) || send_prev;
  const bool invert_new_data = !partial_mode;

  uint8_t bytes_to_send[MAX_TRANSFER_SIZE];

  // Phase 1: previous image via 0x10 (DTM1)
  if (two_pass && this->current_data_index_ < buffer_length) {
    if (this->current_data_index_ == 0) {
      this->command(CMD_DTM1);
    }
    this->start_data_();
    while (this->current_data_index_ < buffer_length) {
      const size_t bytes_to_copy = std::min(MAX_TRANSFER_SIZE, buffer_length - this->current_data_index_);
      for (size_t i = 0; i < bytes_to_copy; i++) {
        const size_t idx = this->current_data_index_ + i;
        bytes_to_send[i] = send_prev ? this->prev_[idx] : static_cast<uint8_t>(~this->buffer_[idx]);
      }
      this->write_array(bytes_to_send, bytes_to_copy);
      this->current_data_index_ += bytes_to_copy;
      if (millis() - start_time > MAX_TRANSFER_TIME) {
        this->disable();
        return false;
      }
    }
    this->disable();
  }

  // Phase 2: new image via 0x13 (DTM2)
  const size_t offset = two_pass ? buffer_length : 0;
  const size_t total = offset + buffer_length;
  if (this->current_data_index_ < total) {
    if (this->current_data_index_ == offset) {
      this->command(CMD_DTM2);
    }
    this->start_data_();
    while (this->current_data_index_ < total) {
      const size_t bytes_to_copy = std::min(MAX_TRANSFER_SIZE, total - this->current_data_index_);
      const size_t data_idx = this->current_data_index_ - offset;
      for (size_t i = 0; i < bytes_to_copy; i++) {
        const uint8_t byte = this->buffer_[data_idx + i];
        bytes_to_send[i] = invert_new_data ? ~byte : byte;
      }
      this->write_array(bytes_to_send, bytes_to_copy);
      this->current_data_index_ += bytes_to_copy;
      if (millis() - start_time > MAX_TRANSFER_TIME) {
        this->disable();
        return false;
      }
    }
    this->disable();
  }

  this->current_data_index_ = 0;
  return true;
}

void EPaperUC8179::power_on() {
  // Power-on is sent at the end of initialise() instead
}

void EPaperUC8179::refresh_screen(bool /*partial*/) {
  ESP_LOGV(TAG, "Refresh");
  this->command(CMD_REFRESH);
  this->next_delay_ = 100;
}

void EPaperUC8179::power_off() {
  ESP_LOGV(TAG, "Power off");
  this->command(CMD_POWER_OFF);
}

}  // namespace esphome::epaper_spi
