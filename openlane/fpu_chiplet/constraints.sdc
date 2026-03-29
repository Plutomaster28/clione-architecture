# FPU chiplet SDC constraints
create_clock -name clk -period 2.50 [get_ports clk]
set_clock_uncertainty 0.10 [get_clocks clk]
set_clock_transition 0.08 [get_clocks clk]

set_input_delay  -clock clk -max 0.4 [all_inputs]
set_input_delay  -clock clk -min 0.1 [all_inputs]
set_output_delay -clock clk -max 0.4 [all_outputs]
set_output_delay -clock clk -min 0.1 [all_outputs]

set_false_path -from [get_ports rst_n]
