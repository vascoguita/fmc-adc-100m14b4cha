vsim -quiet -t 10fs -L unisim work.main

set StdArithNoWarnings 1
set NumericStdNoWarnings 1

run -all
