"""
SPDX-License-Identifier: GPL-3.0-or-later
SPDX-FileCopyrightText: 2020 CERN
"""
import os
import pytest
import time
from PySPEC import PySPEC

@pytest.mark.skipif(pytest.cfg_bitstream is None,
                    reason="We need a bitstream to reflash")
class TestReprogramming(object):
    """
    Collection of regression tests involving carrier reflashing
    """

    @pytest.mark.skipif(pytest.is_spec is False,
                        reason="We need a bitstream to reflash")
    @pytest.mark.repeat(100)
    @pytest.mark.parametrize("size", [100000])
    def test_flat_signal_after_configuration_spec(self, fmc_adc_100m, size):
        """
        The SPEC FPGA could be misconfgured an leading to not acquiring data
        from one of the channels. The problem shows itself with a channel
        delivering only zeros.
        """
        spec = fmc_adc_100m.carrier
        spec.program_fpga(pytest.cfg_bitstream)
        for chan in range(4):
            path = os.path.join(fmc_adc_100m.sys_dev_path,
                                "cset0/chan{:d}/current-value".format(chan))
            sum = 0
            with open(path) as file:
                for i in range(size):
                    file.seek(0)
                    sum += int(file.read())
                    # Should we sleep? It is not the end of the
                    # world if we read twice the same value: the real issue
                    # is that everything is zero
            assert sum != 0, "Missing data on channel {:d}".format(chan)
