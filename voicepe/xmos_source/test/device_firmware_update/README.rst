############################
Check Device Firmware Update
############################

*******
Purpose
*******

Description
===========

This test is a verification of the FFVA device firmware update (DFU) feature.  It verifies that the USB DFU feature works.

Method
======

1. Build and flash the factory firmware.
2. Create an upgrade image
3. DFU the upgrade image
4. Read back the upgrade image and verify the created upgrade image is a bit perfect copy of the upgrade image read.

Inputs
======

N/A

Outputs
=======

Output files are saved in the output directory for manual inspection if necessary.

*************
Running Tests
*************

.. note::

    The Python environment is required to run this test.  See the Requirements section of test/README.rst
    This tests dfu-util commands.  Installation instructions for respective operating system can be found `here <https://dfu-util.sourceforge.net/>`__

Generate dfu-util image files with the following command from the top of the repository:

.. code-block:: console

    bash test/device_firmware_update/check_dfu.sh <firmware> <path-to-output-dir>

All paths must be absolute.  Relative paths may cause errors.

The actions performed by dfu-util can be verified by running a pytest for hash equality:

.. code-block:: console

    pytest test/device_firmware_update/test_dfu.py --readback_image <path-to-output-dir>/readback_upgrade.bin --upgrade_image <path-to-output-dir>/example_ffva_ua_adec_altarch_test_upgrade.bin