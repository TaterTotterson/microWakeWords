import pytest
from pathlib import Path
import filecmp
from orbit.builder.xmos import usb_dfu_upload, usb_dfu_download
import os

from tests.conftest import HW_TESTS

@pytest.mark.skipif( HW_TESTS is None, reason="Hardware tests disabled")
def test_usb_dfu(upgrade_image:Path, usb_dev_id:str) -> None:
    assert( upgrade_image.exists() )

    usb_dfu_upload( usb_dev_id, upgrade_image, reset=False )
    download_file = upgrade_image.with_suffix(".download.bin")
    if download_file.exists():
        os.remove( download_file )
    usb_dfu_download( usb_dev_id, download_file ) 
    assert( filecmp.cmp(upgrade_image, download_file) )





    