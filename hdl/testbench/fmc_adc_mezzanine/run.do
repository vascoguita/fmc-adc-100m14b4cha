vsim -quiet -L unisim work.main -voptargs=+acc

set StdArithNoWarnings 1
set NumericStdNoWarnings 1

radix -hexadecimal

log -r /*

run -all

wave zoomfull
