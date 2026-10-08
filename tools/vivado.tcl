# Usage: vivado -mode batch -source tools/vivado.tcl -tclargs <FPGA-part>
# Run from project root. Generic synthesis only, no board implementation.
if {$argc != 1} { error "Specify target part, e.g. xc7a100tcsg324-1" }
file mkdir results
read_verilog rtl/Single_Cycle.v
synth_design -top Single_Cycle -part [lindex $argv 0]
report_utilization -file results/vivado_utilization.rpt
report_timing_summary -file results/vivado_timing.rpt
write_checkpoint -force results/core.dcp
