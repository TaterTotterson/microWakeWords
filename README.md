<div align="center">
  <a href="https://taterassistant.com">
    <img src="images/tater-repo-logo.png" alt="microWakeWords" width="460"/>
  </a>
</div>
<h3 align="center">
  <a href="https://taterassistant.com">taterassistant.com</a>
</h3>

---

## Repository Archived

This repository is no longer the active firmware home for Tater voice satellites.

Tater has moved away from ESPHome-based satellite firmware and now uses dedicated native firmware built specifically for Tater hardware. That change lets Tater provide a more polished satellite experience with direct pairing, native device settings, streaming audio, firmware updates, diagnostics, display support, LED control, intercom, timers, and tighter coordination between satellites.

The current firmware project lives here:

**[Tater Native Firmware](https://github.com/TaterTotterson/Tater-Native-Firmware)**

## What This Means

The ESPHome/Home Assistant satellite firmware in this repository is no longer maintained as the supported Tater satellite path. New Tater satellite development happens in the native firmware repository instead.

This repository should be treated as historical reference only. It is not the recommended way to build, flash, or configure Tater satellites.

## Home Assistant Users

The old firmware path in this repository is no longer supported as a Home Assistant voice satellite setup.

If you still want to use Home Assistant with Tater satellites, use the satellites through Tater Native Firmware, then connect Home Assistant to Tater. In that setup, Tater manages the satellites and voice pipeline, while Home Assistant can still be used alongside Tater for smart-home devices, automations, and control.

Start here:

- **Tater:** [https://github.com/TaterTotterson/Tater](https://github.com/TaterTotterson/Tater)
- **Tater Native Firmware:** [https://github.com/TaterTotterson/Tater-Native-Firmware](https://github.com/TaterTotterson/Tater-Native-Firmware)

## Status

Archived. No new wake-word requests, ESPHome YAML updates, or Home Assistant satellite firmware changes are planned here.
