vsim -quiet -t 10fs -L unisim work.main -voptargs=+acc -suppress 143,1270,8617,8683,8684,8822

set StdArithNoWarnings 1
set NumericStdNoWarnings 1

radix -hexadecimal

log -r /*

run -all

wave zoomfull
