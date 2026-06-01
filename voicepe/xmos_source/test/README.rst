#################
XCORE-VOICE Tests
#################

************
Requirements
************

The following software is required for running tests.

* Linux, MacOS, or Windows Subsystem for Linux
* `Python 3.8 <https://www.python.org/>`__

Python Virtual Environment
==========================

While not required, we recommend you setup a Python virtual environment, like `https://realpython.com/python-virtual-environments-a-primer/`

Install Python Packages
=======================

Install development packages with the Python virtual environment activated:

.. code-block:: console

    pip install -r test/requirements.txt

**************************
Building Host Applications
**************************

To build the host applications, run the following command from the top of the repository: 

.. code-block:: console

    bash tools/ci/build_host.sh

The ```build_host.sh``` script will copy the host applications to the ``dist_host`` folder.

**********************
Building Test Firmware
**********************

To build the test application firmware and filesystem files, run the following command from the top of the repository: 

.. code-block:: console

    bash tools/ci/build_tests.sh

The ``build_test.sh`` script will copy the test applications and filesystem files to the ``dist`` folder.

*************
Running Tests
*************

Tests exists for the following:

- Audio processing pipelines
- Speech recognition command dictionaries
- Sample rate conversion
- DFU
- GPIO
- Low power mode's audio ring buffer

To run tests, see the README files located in the directories containing each test group.
