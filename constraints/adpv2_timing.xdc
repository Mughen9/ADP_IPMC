# ============================================================
# ADPv2 Timing Constraints
# Target: Xilinx Artix-7 xc7a200t-3
# Tool:   Vivado 2025.1
#
# Post-route results:
#   WNS = +0.053 ns  (timing closed)
#   TNS = 0.000 ns
#   Clock = 98 MHz (10.2 ns constraint)
# ============================================================

# Primary clock constraint — 98 MHz (10.2 ns period)
create_clock -period 10.200 -name clk [get_ports clk]

# Input delay constraints
set_input_delay  -clock clk -max 2.0 [get_ports {pix_data* pix_valid start}]
set_input_delay  -clock clk -min 0.5 [get_ports {pix_data* pix_valid start}]

# Output delay constraints
set_output_delay -clock clk -max 2.0 [get_ports {y_out* y_valid done}]
set_output_delay -clock clk -min 0.5 [get_ports {y_out* y_valid done}]
