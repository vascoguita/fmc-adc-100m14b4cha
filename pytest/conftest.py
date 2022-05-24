"""
SPDX-License-Identifier: GPL-3.0-or-later
SPDX-FileCopyrightText: 2022 CERN
"""

import os
import pytest
from PySPEC import PySPEC
import re
import socket


def get_carrier(dev_id, fec):
    sys_dev_path = os.path.join("/sys/bus/zio/devices/",
                                "adc-100m14b-{:04x}".format(dev_id))
    if fec is None:
        path = os.readlink(sys_dev_path)
    else:
        ret = os.popen("ssh -T {} 'readlink -f {}'".format(fec, sys_dev_path))
        path = ret.read().strip()
        ret.close()

    r = re.search(r".*/(spec)", path)
    if r is None:
        Exception("Can't understand carrier type. Supported: 'spec'")
    return r.group(1)


class FmcAdc100M():
    def __init__(self, dev_id):
        self.dev_id = dev_id
        self.sys_dev_path = os.path.join("/sys/bus/zio/devices/",
                                         "adc-100m14b-{:04x}".format(self.dev_id))

        path = os.path.abspath(os.path.join(os.path.dirname(self.sys_dev_path),
                                            os.readlink(self.sys_dev_path)))
        r = re.search(r"/sys/devices/.*/(spec|svec-vme)[-.](([0-9a-f]{4}:([0-9a-f]{2}:[0-9a-f]{2}.[0-9a-f]))|[0-9]+)/",
                      path)
        if r is None:
            Exception("Can't understand carrier type. Supported: 'spec'")

        if r.group(1) == "spec":
            self.carrier = PySPEC(r.group(4))
        elif r.group(1) == "svec-vme":
            Exception("TODO implement SVEC support")
        else:
            Exception("Can't understand carrier type. Bad regular expression")


@pytest.fixture(scope="module")
def fmc_adc_100m():
    dev = FmcAdc100M(pytest.dev_id)
    yield dev


def pytest_addoption(parser):
    parser.addoption("--adc-id", type=lambda x: int(x, 0),
                     required=True, help="ADC Identifier")
    parser.addoption("--bitstream",
                     default=None, help="SPEC bitstream to be tested")
    parser.addoption("--fec",
                     default=None, help="Remote FEC to be tested")


def pytest_configure(config):
    pytest.dev_id = config.getoption("--adc-id")
    pytest.cfg_bitstream = config.getoption("--bitstream")
    pytest.fec = config.getoption("--fec")

    pytest.is_fec = socket.gethostname()[:2] == "cf"
    assert pytest.is_fec ^ (pytest.fec is not None)
    pytest.is_spec = get_carrier(pytest.dev_id, pytest.fec) == "spec"
