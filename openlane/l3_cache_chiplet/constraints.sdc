# L3 cache chiplet SDC constraints
create_clock -name clk -period 4.00 [get_ports clk]
set_clock_uncertainty 0.15 [get_clocks clk]
set_clock_transition 0.12 [get_clocks clk]

set_input_delay  -clock clk -max 0.6 [all_inputs]
set_input_delay  -clock clk -min 0.1 [all_inputs]
set_output_delay -clock clk -max 0.6 [all_outputs]
set_output_delay -clock clk -min 0.1 [all_outputs]

set_false_path -from [get_ports rst_n]
