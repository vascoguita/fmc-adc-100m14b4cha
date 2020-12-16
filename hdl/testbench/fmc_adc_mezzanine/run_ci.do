# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

vsim -quiet -L unisim work.main

set StdArithNoWarnings 1
set NumericStdNoWarnings 1

run -all
