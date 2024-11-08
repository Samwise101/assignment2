#include "hls_design_meta.h"
const Port_Property HLS_Design_Meta::port_props[]={
	Port_Property("ap_clk", 1, hls_in, -1, "", "", 1),
	Port_Property("ap_rst_n", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_AWVALID", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_AWREADY", 1, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_AWADDR", 10, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_WVALID", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_WREADY", 1, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_WDATA", 32, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_WSTRB", 4, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_ARVALID", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_ARREADY", 1, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_ARADDR", 10, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_RVALID", 1, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_RREADY", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_RDATA", 32, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_RRESP", 2, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_BVALID", 1, hls_out, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_BREADY", 1, hls_in, -1, "", "", 1),
	Port_Property("s_axi_AXI_CPU_BRESP", 2, hls_out, -1, "", "", 1),
	Port_Property("interrupt", 1, hls_out, -1, "", "", 1),
};
const char* HLS_Design_Meta::dut_name = "filterImage";
