<div align="center">
  <a href="https://taterassistant.com">
    <img src="images/tater-repo-logo.png" alt="Tater S3Box Display" width="460"/>
  </a>
</div>
<h3 align="center">
  <a href="https://taterassistant.com">taterassistant.com</a>
</h3>

# Tater ESP32-S3-BOX-3 Display

ESPHome firmware for the ESP32-S3-BOX-3 display that talks to Tater directly instead of binding the screen to Home Assistant widgets.

The screen is built with LVGL and uses a high-contrast Tater color language: black surfaces, bright white text, clear muted text, and vivid orange accents.

## Screen Preview

![Tater ESP32-S3-BOX-3 display mockup](docs/s3box-display-preview.png)

## What It Shows

- A Tater home dashboard with time, date, connection status, indoor/outdoor temperature, humidity, wind, rain, and lightning slots.
- A transient notification page for display events pushed through Tater.
- A camera snapshot notification page that downloads the event image and shows it with the description text.
- A dedicated tool-call page for live voice tool progress, using the mirrored-dot animation language from the Tater voice firmware.
- ESP32-S3-BOX-3 microphone, speaker, and wake-word voice pipeline support.

## Tater API

The firmware polls these Tater endpoints:

```text
GET /tater-ha/v1/display/feed
GET /tater-ha/v1/display/events
```

If Tater voice/display API auth is enabled, the Tater firmware flasher injects the display token during build.

## Setup

Use the Tater firmware flasher in Tater's Tater Voice/Firmware tab.

Choose the Tater S3Box Display firmware, pick the target display, then flash the prebuilt firmware over OTA or browser USB. Weather/display sensor slots are managed live in Tater after the firmware is installed, so changing sensors does not require another firmware build.

## Requirements

- ESPHome 2026.4.0 or newer
- ESP32-S3-BOX-3 with PSRAM
- Tater running the display feed/event API

## Notes

- The display uses LVGL with ESPHome's `mipi_spi` S3BOX display driver, `auto_clear_enabled: false`, and `update_interval: never`.
- The ESPHome node keeps the locked `taters3box` base name and enables `name_add_mac_suffix` so each flashed display gets a unique hostname.
- The Box-3 speaker path includes the ES8311 DAC, I2S speaker, speaker media player, and GPIO46 speaker-enable pin from the upstream Box-3 voice assistant example.
- Tater polling waits for Wi-Fi and uses single-run scripts so a missing network or offline Tater API cannot overlap HTTP requests during boot.
- The firmware exposes a `Refresh Display Events` ESPHome button so Tater can nudge connected displays to fetch fresh display feed data and events immediately while the normal poll remains as a fallback.
- Fonts use ESPHome's Google Fonts support so the Tater firmware builder can compile the fetched YAML without extra local assets.
- Camera snapshot display events use ESPHome `online_image`; Tater-hosted Awareness snapshots are served as JPEGs and fetched with the display API token when configured.
