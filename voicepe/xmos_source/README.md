# VoicePE XMOS Firmware Source

This folder is the Git-trackable source copy for the VoicePE XMOS firmware used by the VoicePE ESPHome package.

The ignored local build workspace remains:

```bash
voicepe/xmos_firmware/
```

Use that ignored folder for local experiments and builds. When an XMOS change is proven good, sync the source change back into this folder and copy the resulting upgrade binary into:

```bash
voicepe/firmware/xmos/
```

## What Is Included

- VoicePE FFVA application source under `src/`
- Top-level CMake/build files
- XMOS CMake toolchain helper
- Small first-party modules under `modules/asr/` and `modules/audio_pipelines/`
- Wrapper CMake files needed around external XMOS dependency modules
- Dependency patches under `patches/`
- Upstream README preserved as `UPSTREAM_README.md`

## What Is Not Included

This folder intentionally excludes generated and heavy local files:

- `build/`
- `.venv/`
- `.git/`
- generated `.bin`, `.xe`, `.fs`, `.o`, and similar outputs
- external XMOS dependency module checkouts

The dependency commit pins are recorded in `submodules.lock`. To materialize the external XMOS dependencies inside this folder and apply local dependency patches, run:

```bash
bash scripts/fetch_dependencies.sh
```

Then build the fixed-delay upgrade image with:

```bash
./build_voicepe_fixed_delay.sh
```

The build script expects XMOS XTC Tools to be installed. Set `XMOS_TOOL_PATH` if XTC is not installed at `/Applications/XMOS_XTC_15.3.1`.

## Current Source Base

This source started from `esphome/voice-kit-xmos-firmware` at:

```text
ef04d4b Pin GitHub Actions to commit SHAs (#9)
```

The current source includes the VoicePE DoA/VoD XMOS additions used by the ESPHome VoicePE package.
