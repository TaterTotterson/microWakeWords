# Prebuilt Firmware

Versioned ESPHome release binaries live under folders like `3.0.4/`.

- Use `*.ota.bin` for Tater app / ESPHome OTA updates.
- Use `*.factory.bin` for first USB/serial flash or recovery flashes at offset `0x0`.
- Each version folder includes a `manifest.json` with device keys, artifact paths, sizes, and SHA-256 checksums.
- `latest.json` points clients to the current versioned manifest.
