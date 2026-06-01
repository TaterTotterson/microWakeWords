# Sat1 XMOS Firmware Source

This folder is the Git-trackable source copy for the Satellite1 XMOS firmware used by the Sat1 ESPHome package.

The ignored local build workspace remains:

```bash
sat1/xmos_firmware/
```

Use that ignored folder for local experiments and builds. When an XMOS change is proven good, sync the source change back into this folder and copy the resulting factory binary into:

```bash
sat1/firmware/xmos/
```

## What Is Included

- Satellite1 application source: `satellite-xmos-firmware/`
- Top-level CMake/build files
- XMOS CMake toolchain helper
- Custom FPH support modules under `modules/fph/`
- Wrapper CMake files needed around external XMOS dependency modules
- Upstream README preserved as `UPSTREAM_README.md`

## What Is Not Included

This folder intentionally excludes generated and heavy local files:

- `build/`
- `.venv/`
- `.git/`
- generated `.bin`, `.xe`, `.fs`, `.o`, and similar outputs
- external XMOS dependency module checkouts

The dependency commit pins are recorded in `submodules.lock`. To materialize the external XMOS dependencies inside this folder, run:

```bash
bash scripts/fetch_dependencies.sh
```

Then build with:

```bash
./build_sat1_fixed_delay.sh
```

The build script expects XMOS XTC Tools to be installed. Set `XMOS_TOOL_PATH` if XTC is not installed at `/Applications/XMOS_XTC_15.3.1`.

## Current Source Base

This source started from `FutureProofHomes/Satellite1-XMOS` at:

```text
86cf1ee Merge pull request #106 from FutureProofHomes/align_branches
```

The current source includes the Sat1 DoA/VoD XMOS additions used by the ESPHome Sat1 package.
