# Rebuild only; this script never programs a device.
set root [file normalize [file join [file dirname [info script]] ..]]
set output [file join $root build generated]
if {[info exists ::env(GPSDO_BUILD_DIR)]} {set output [file normalize $::env(GPSDO_BUILD_DIR)]}
open_project [file join $output gpsdo.xpr]
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
set report_dir [file join $output reports]
file mkdir $report_dir
report_utilization -file [file join $report_dir utilization.rpt]
report_timing_summary -report_unconstrained -file [file join $report_dir timing.rpt]
write_hw_platform -fixed -include_bit -force -file [file join $output gnss_bd_wrapper.xsa]
puts "GPSDO_IMPLEMENTATION_COMPLETE"
close_project

