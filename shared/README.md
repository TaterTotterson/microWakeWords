Shared firmware dependencies

This folder stores vendored dependencies that multiple Tater firmware YAMLs use.
The goal is to reduce build breakage from upstream component or asset changes.

Included here:
- ESPHome external components shared by multiple firmwares.
- Fixed firmware blobs used by device-side flash/update actions.
- Audio assets used by `audio_file`.
- Wake-word model assets that were previously pulled from third-party releases.

Not included here:
- ESPHome itself.
- PlatformIO packages, ESP-IDF, and compiler toolchains.
- Runtime services such as remote wake-word or OpenWakeWord servers.

Those should be pinned and tested separately when changing ESPHome versions.
