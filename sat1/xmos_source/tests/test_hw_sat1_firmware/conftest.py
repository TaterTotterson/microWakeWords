import pytest
from pathlib import Path

from orbit.builder.xmos import build_firmware, reset_device
from orbit.testing.xmos import run_firmware
from orbit import BOARD_XTAG_IDS, BOARD_USB_IDS
from tests import PROJ_ROOT
from tests.conftest import HW_TESTS

import time

BOARD = "Satellite1"
VARIANT = "satellite1_firmware_fixed_delay"
BUILD_DIR = "build_test_satellite1_firmware_fixed_delay"

@pytest.fixture(scope="session", autouse=True)
def setup_firmware():
    """Build Satellite1 firmware and run it using xrun."""
    
    if HW_TESTS != "build_and_xrun":
        yield
        return
        
    try:
        build_firmware( 
            build_dir=BUILD_DIR,
            src_dir=PROJ_ROOT,
            variant=f"create_upgrade_img_{VARIANT}",
            clean=False,
            defines=[]
        )
    except Exception as e:
        pytest.exit(f"Firmware build failed: {e}")

    target = PROJ_ROOT / BUILD_DIR / f"{VARIANT}.upgrade.bin"
    if not target.exists():
         pytest.exit(f"Firmware build failed, target not found: {target}")
    
    print("Running firmware with xrun...")
    target_xe = PROJ_ROOT / BUILD_DIR / f"{VARIANT}.xe"
    
    reset_device(BOARD_XTAG_IDS[BOARD])
    pid = run_firmware(
        firmware=target_xe,
        xtag_id=BOARD_XTAG_IDS[BOARD],
    )
    
    # Wait for firmware to start
    time.sleep(5)
    
    yield  # Tests execute here
    
    print("Stopping xrun...")
    pid.terminate()


@pytest.fixture(scope="session")
def upgrade_image() -> Path:
    return PROJ_ROOT / BUILD_DIR / f"{VARIANT}.upgrade.bin"

@pytest.fixture(scope="session")
def xtag_id() -> str:
    return BOARD_XTAG_IDS[BOARD]

@pytest.fixture(scope="session")
def usb_dev_id() -> str:
    return BOARD_USB_IDS[BOARD]