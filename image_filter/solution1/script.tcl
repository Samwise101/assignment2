############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
############################################################
open_project image_filter
set_top filterImage
add_files image_filter/image_filter.hpp
add_files image_filter/image_filter.cpp
add_files -tb image_filter/image_filter_tb.cpp -cflags "-Wno-unknown-pragmas" -csimflags "-Wno-unknown-pragmas"
open_solution "solution1" -flow_target vivado
set_part {xczu3eg-sbva484-1-e}
create_clock -period 10 -name default
config_export -format ip_catalog -output /home/samuel/MpSoc4Drones/ip/filterImage.zip -rtl vhdl
source "./image_filter/solution1/directives.tcl"
csim_design
csynth_design
cosim_design
export_design -rtl vhdl -format ip_catalog -output /home/samuel/MpSoc4Drones/ip/filterImage.zip
