open_project [file join [file dirname [info script]] acg720_one_euro_WT01E2.gprj]
set_option -top_module ab4_top
set_option -verilog_std v2001
set_option -bit_security 0
# Gowin SUG1220: timing-friendly alternate placement/routing and fanout replication.
run all
exit
