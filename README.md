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

This repository is no longer the active firmware home for satellites when they are used with Tater.

Tater has moved away from ESPHome-based satellite firmware and now uses dedicated native firmware for supported open satellite hardware such as VoicePE, Sat1, ReSpeaker devices, and S3 Box display satellites. Those devices are not Tater-only hardware; they can still be used with Home Assistant or other projects when flashed with firmware made for those systems.

For Tater, the native firmware path lets Tater provide a more polished satellite experience with direct pairing, native device settings, streaming audio, firmware updates, diagnostics, display support, LED control, intercom, timers, and tighter coordination between satellites.

The current firmware project lives here:

**[Tater Native Firmware](https://github.com/TaterTotterson/Tater-Native-Firmware)**

## What This Means

The ESPHome/Home Assistant satellite firmware in this repository is no longer maintained as the supported firmware path for using these satellites with Tater. New Tater satellite development happens in the native firmware repository instead.

This repository should be treated as historical reference only. It is not the recommended way to build, flash, or configure satellites for Tater.

## Home Assistant Users

The old firmware path in this repository is no longer maintained by Tater as a Home Assistant voice satellite setup.

If you still want to use Home Assistant with the same satellite hardware, you can flash firmware intended for Home Assistant. If you want the Tater native satellite experience and still want Home Assistant in the loop, use Tater Native Firmware on the satellites, then connect Home Assistant to Tater. In that setup, Tater manages the satellites and voice pipeline, while Home Assistant can still be used alongside Tater for smart-home devices, automations, and control.

Start here:

- **Tater:** [https://github.com/TaterTotterson/Tater](https://github.com/TaterTotterson/Tater)
- **Tater Native Firmware:** [https://github.com/TaterTotterson/Tater-Native-Firmware](https://github.com/TaterTotterson/Tater-Native-Firmware)

## Status

Archived. No new wake-word requests, ESPHome YAML updates, or Home Assistant satellite firmware changes are planned here.
