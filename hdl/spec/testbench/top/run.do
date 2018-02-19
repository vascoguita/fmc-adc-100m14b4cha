vsim -t 1ps -L unisim -L XilinxCoreLib work.main -novopt
set StdArithNoWarnings 1
set NumericStdNoWarnings 1
do wave.do
radix -hexadecimal
run 70us
wave zoomfull
radix -hexadecimal
