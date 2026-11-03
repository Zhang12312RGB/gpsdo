# Vivado 2026.1. Recreate without touching the original development project.
# Usage: vivado -mode batch -source build/create_project.tcl
set root [file normalize [file join [file dirname [info script]] ..]]
set output [file join $root build generated]
if {[info exists ::env(GPSDO_BUILD_DIR)]} {set output [file normalize $::env(GPSDO_BUILD_DIR)]}
if {[file exists [file join $output gpsdo.xpr]]} {error "Output project exists: choose a new GPSDO_BUILD_DIR"}
create_project gpsdo $output -part xc7z010clg400-1
set_property target_language Verilog [current_project]
add_files -norecurse [glob [file join $root src rtl *.v]]
add_files -fileset constrs_1 -norecurse [file join $root src constrs gnss_gpsdo_ax7010.xdc]
update_compile_order -fileset sources_1
source [file join $root src bd gnss_bd.tcl]
validate_bd_design
save_bd_design
set bd [lindex [get_files *.bd] 0]
generate_target all $bd
set wrapper [make_wrapper -files $bd -top]
add_files -norecurse $wrapper
set_property top gnss_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1
puts "GPSDO_PROJECT_READY [get_property DIRECTORY [current_project]]"
close_project

