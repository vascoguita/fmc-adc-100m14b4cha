vsim -quiet -t 10fs -L unisim work.main -novopt

set StdArithNoWarnings 1
set NumericStdNoWarnings 1

radix -hexadecimal

log -r /*

run -all

wave zoomfull
