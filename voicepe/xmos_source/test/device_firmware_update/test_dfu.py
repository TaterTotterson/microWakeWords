# Copyright 2022-2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.
# XMOS Public License: Version 1

import hashlib as hl

BUF_SIZE = 65536

def test_readback(upgrade_image, readback_image):
    digests = []
    for bin_file in [upgrade_image, readback_image]:
        a = hl.sha1()
        with open(bin_file, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                a.update(data)
        digests.append(a.hexdigest())

    assert (digests[0] == digests[1])