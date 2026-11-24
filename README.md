# esphome-epaper-persist

ESPHome external component: a patched copy of the built-in `epaper_spi`
component (ESPHome **2026.11.0**). The UC8179 driver (Waveshare 7.5" V2,
TRMNL 7.5" OG DIY kit) can now do **partial refresh after deep sleep**.

Upstream, the controller's previous-image RAM is lost on deep sleep, so the
first update after every wake-up is a full refresh. With
`full_update_every > 1`, this patch:

- stores the last frame shown in NVS after every update (RLE compressed,
  ~13 KB for a typical dashboard), and the refresh counter in RTC memory;
- after a wake-up from deep sleep, restores the frame and sends it as the
  previous image (DTM1), so the update is a partial refresh with no flashing;
- every `full_update_every`-th update is still a fast full refresh, and so is
  the first update after power loss or flashing.

Only `epaper_spi_uc8179.h` / `epaper_spi_uc8179.cpp` differ from upstream.

```yaml
external_components:
  - source: github://memingr/esphome-epaper-persist
    components: [epaper_spi]

display:
  - platform: epaper_spi
    model: Waveshare-7.5in-V2
    full_update_every: 12
    update_interval: never
    # ...
```

Wait for `!id(epd).is_updating()` before `deep_sleep.enter`.

Based on ESPHome (https://github.com/esphome/esphome): the Python code is
MIT-licensed and the C++ code is GPLv3, and this copy keeps the same licenses.
After upgrading ESPHome, re-sync the folder from upstream and re-apply the
patch.
