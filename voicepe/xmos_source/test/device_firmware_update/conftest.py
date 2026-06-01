# Copyright 2022-2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.
# XMOS Public License: Version 1

def pytest_addoption(parser):
    parser.addoption("--upgrade_image", action="store", default="upgrade")
    parser.addoption("--readback_image", action="store", default="readback")

def pytest_generate_tests(metafunc):
    option_value = metafunc.config.option.upgrade_image
    if 'upgrade_image' in metafunc.fixturenames and option_value is not None:
        metafunc.parametrize("upgrade_image", [option_value])
        
    option_value = metafunc.config.option.readback_image
    if 'readback_image' in metafunc.fixturenames and option_value is not None:
        metafunc.parametrize("readback_image", [option_value])