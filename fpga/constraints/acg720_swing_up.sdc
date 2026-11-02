# Physical oscillator remains50MHz/20ns. Require0.5ns design reserve by19.5ns constraint.
// Actual board oscillator remains 50MHz. Tighter 19.5ns constraint adds margin.
create_clock -name clk_50m -period 19.5 -waveform {0.000 9.500} [get_ports {clk_50m}]
