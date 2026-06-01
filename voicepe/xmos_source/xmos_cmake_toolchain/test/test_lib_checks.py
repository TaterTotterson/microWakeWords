# Copyright 2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.

"""Tests that check the contents of the files meet our standards"""
from subprocess import run
from pathlib import Path
import json
import re

REPO_ROOT = (Path(__file__).parent/"..").resolve()

def test_source_check():
    """
    runs xmos_source_check and asserts on the exit code

    To fix a failure run `xmos_source_check update . xmos_public_v1` from the repo root.
    """
    ret = run(f"xmos_source_check check {REPO_ROOT} xmos_public_v1".split())
    assert 0 == ret.returncode

def test_license_check():
    """
    runs xmos_license_check and asserts on the exit code
    """
    ret = run(f"xmos_license_check check {REPO_ROOT} xmos_public_v1".split())
    assert 0 == ret.returncode

def test_changelog_check():
    """
    checks changelog conforms with the standards
    """
    ret = run(f"xmos_changelog_check check {REPO_ROOT}".split())
    assert 0 == ret.returncode
