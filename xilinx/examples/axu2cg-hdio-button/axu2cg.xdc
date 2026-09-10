# Onboard KEY1 and LED3 are in PL HDIO bank 24 at 3.3 V.
set_property PACKAGE_PIN AA13 [get_ports key1_n]
set_property IOSTANDARD LVCMOS33 [get_ports key1_n]

set_property PACKAGE_PIN AA12 [get_ports led]
set_property IOSTANDARD LVCMOS33 [get_ports led]
set_property DRIVE 8 [get_ports led]
set_property SLEW SLOW [get_ports led]
