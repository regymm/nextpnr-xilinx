set example_dir [file normalize [lindex $argv 0]]
set output_dir [file join $example_dir vivado-build]
set part xczu2cg-sfvc784-1-e

file mkdir $output_dir
create_project -force -part $part blinky [file join $output_dir project]
read_verilog [file join $example_dir top.v]
read_xdc [file join $example_dir axu2cg.xdc]

synth_design -top top -part $part
opt_design
place_design
route_design

set_property BITSTREAM.GENERAL.COMPRESS FALSE [current_design]
set_property BITSTREAM.GENERAL.PERFRAMECRC NO [current_design]

report_utilization -file [file join $output_dir utilization.rpt]
report_route_status -file [file join $output_dir route_status.rpt]
write_checkpoint -force [file join $output_dir vivado_routed.dcp]
write_bitstream -force [file join $output_dir vivado.bit]
