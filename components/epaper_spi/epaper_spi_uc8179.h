#pragma once

#include <vector>

#include "epaper_spi_ultrachip.h"

namespace esphome::epaper_spi {

/**
 * Monochrome e-paper displays using the UC8179 controller.
 * Supports: 7.5" V2 (EPD_7in5_V2), 800x480 pixels, as used by the
 * Waveshare 7.5" V2 HAT and the Seeed reTerminal E1001.
 *
 * Buffer layout: 1 bit per pixel, 1=white, 0=black (the base class default).
 *
 * Three refresh modes are used, following the Waveshare EPD_7in5_V2 examples:
 * - full_update_every == 1: plain full refresh.
 * - full_update_every > 1, full update: fast full refresh (DTM1 = inverse, DTM2 = new).
 * - full_update_every > 1, partial update: partial refresh.
 *
 * LOCAL PATCH (TRMNL / deep sleep):
 * With full_update_every > 1 the last frame shown is kept in NVS (run-length
 * encoded) and the partial-update counter in RTC memory. After waking from
 * deep sleep the frame is restored and sent as the previous image (DTM1), so
 * the first update after wake-up can be a partial one instead of a full
 * refresh. After a cold boot (power loss, flashing) the first update is full.
 */
class EPaperUC8179 final : public EPaperUltraChip {
 public:
  EPaperUC8179(const char *name, uint16_t width, uint16_t height, const uint8_t *init_sequence,
               size_t init_sequence_length)
      : EPaperUltraChip(name, width, height, init_sequence, init_sequence_length, DISPLAY_TYPE_BINARY) {}

  void setup() override;

 protected:
  bool initialise(bool partial) override;
  bool transfer_data() override;
  void refresh_screen(bool partial) override;
  void power_on() override;
  void power_off() override;
  void deep_sleep() override;
  void set_refresh_mode_();

  // previous frame persistence
  bool load_frame_();
  void store_frame_();
  std::vector<uint8_t> prev_{};  // the frame currently on the panel, empty when unknown
};

}  // namespace esphome::epaper_spi
