####################
Documentation Source
####################

This folder contains source files for the documentation and is intended for XMOS employees. Pre-built documentation is published on `the XMOS website<https://www.xmos.com>`_.
The sources do not render well in GitHub or an RST viewer. In addition, some information is not visible at all and some links will not be functional.

**********************
Building Documentation
**********************

=============
Prerequisites
=============

Use the `xmosdoc tool <https://github.com/xmos/xmosdoc>`_ either via docker or install it into a pip environment.

========
Building
========

To build the documentation, run the following command in the root of the repository:

.. code-block:: console

    # via pip package
    xmosdoc clean html latex
    # via docker
    $ docker run --rm -t -u "$(id -u):$(id -g)" -v $(pwd):/build ghcr.io/xmos/xmosdoc clean html latex

HTML document output is saved in the ``doc/_build/html`` folder.  Open ``index.html`` to preview the saved documentation.

Please refer to the ``xmosdoc`` documentation for a complete guide on how to use the tool.

**********************
Adding a New Component
**********************

Follow the following steps to add a new component.

- Add an entry for the new component's top-level document to the appropriate TOC in the documents tree.
- If the new component uses `Doxygen`, append the appropriate path(s) to the INPUT variable in `Doxyfile.inc`.
- If the new component includes `.rst` files that should **not** be part of the documentation build, append the appropriate pattern(s) to `exclude_patterns.inc`.

***
FAQ
***

Q: Is it possible to build just a subset of the documentation?

A: Yes, however it is not recommended at this time.

Q: Is it possible to used the ``livehtml`` feature of Sphinx?

A: Yes, run xmosdoc with the ``--auto`` option.

Q: Where can I learn more about the XMOS ``xmosdoc`` tools?

A: See the https://github.com/xmos/xmosdoc repository.  See the ``xmosdoc`` repository README for details on additional build options.

Q: How do I suggest enhancements to the XMOS ``xmosdoc`` tool?

A: Create a new issue here: https://github.com/xmos/xmosdoc/issues
