# 25 MHz PL reference clock, bank 44 at 1.8 V.
set_property PACKAGE_PIN AB11 [get_ports pl_ref_clk]
set_property IOSTANDARD LVCMOS18 [get_ports pl_ref_clk]
create_clock -period 40.000 [get_ports pl_ref_clk]

# Onboard active-high LED4, bank 24 at 3.3 V.
set_property PACKAGE_PIN AB13 [get_ports led]
set_property IOSTANDARD LVCMOS33 [get_ports led]
set_property DRIVE 8 [get_ports led]
set_property SLEW SLOW [get_ports led]
