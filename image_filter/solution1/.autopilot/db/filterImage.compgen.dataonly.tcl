# This script segment is generated automatically by AutoPilot

set axilite_register_dict [dict create]
set port_AXI_CPU {
rowBelow { 
	dir I
	width 8
	depth 120
	mode ap_memory
	offset 128
	offset_end 255
}
rowCenter { 
	dir I
	width 8
	depth 120
	mode ap_memory
	offset 256
	offset_end 383
}
rowAbove { 
	dir I
	width 8
	depth 120
	mode ap_memory
	offset 384
	offset_end 511
}
outputRow { 
	dir O
	width 8
	depth 120
	mode ap_memory
	offset 512
	offset_end 639
}
ap_start { }
ap_done { }
ap_ready { }
ap_idle { }
}
dict set axilite_register_dict AXI_CPU $port_AXI_CPU


