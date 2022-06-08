"""
SPDX-License-Identifier: GPL-3.0-or-later
SPDX-FileCopyrightText: 2022 CERN
"""
import json
import os
import pytest
import time
from PySPEC import PySPEC

@pytest.mark.skipif(pytest.cfg_bitstream is None,
                    reason="We need a bitstream to reflash")
class TestFlatSignal(object):
    """
    Collection of regression tests involving carrier reflashing
    """

    @pytest.mark.skipif(pytest.is_spec is False,
                        reason="We need a bitstream to reflash")
    @pytest.mark.repeat(100)
    @pytest.mark.parametrize("size", [100000])
    def test_fpga_reconfiguration_spec(self, fmc_adc_100m, size):
        """
        The SPEC FPGA could be misconfgured an leading to not acquiring data
        from one of the channels. The problem shows itself with a channel
        delivering only zeros.
        """
        spec = fmc_adc_100m.carrier
        spec.program_fpga(pytest.cfg_bitstream)
        pattern = 0x555
        fmc_adc_100m.pattern_data = pattern
        for chan in range(4):
            path = os.path.join(fmc_adc_100m.sys_dev_path,
                                "cset0/chan{:d}/current-value".format(chan))
            sum = 0
            with open(path) as file:
                for i in range(size):
                    file.seek(0)
                    value = int(file.read())
                    assert (value >> 2) == pattern
                    sum += value
                    # Should we sleep? It is not the end of the
                    # world if we read twice the same value: the real issue
                    # is that everything is zero
            assert sum != 0, "Missing data on channel {:d}".format(chan)

    @pytest.mark.skipif(pytest.is_fec is True,
                        reason="We must be NOT on a FEC")
    @pytest.mark.parametrize("fec", [pytest.fec])
    @pytest.mark.parametrize("dev_id", [pytest.dev_id])
    @pytest.mark.repeat(100)
    def test_reboot(self, fec, dev_id):
        os.system("ssh -T {} 'sudo reboot'".format(fec))
        time.sleep(90)

        tool = "/acc/local/L867/drv/adc-lib/4.0.3/bin/adc-acq"
        cmd = "sudo {} -D fmc-adc-100m14b4cha@0x{:x} -a 0,1000,1 --stat -s 0 --trg-sw 1".format(tool, dev_id)
        ret = os.popen("ssh -T {} '{}'".format(fec, cmd))
        data = json.loads(ret.read().strip())
        for chan in data["statistics"]:
            assert chan["average"] != 0
