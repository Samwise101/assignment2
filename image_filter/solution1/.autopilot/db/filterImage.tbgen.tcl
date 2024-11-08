set moduleName filterImage
set isTopModule 1
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set pipeline_type none
set FunctionProtocol ap_ctrl_hs
set isOneStateSeq 0
set ProfileFlag 0
set StallSigGenFlag 0
set isEnableWaveformDebug 1
set C_modelName {filterImage}
set C_modelType { void 0 }
set C_modelArgList {
	{ rowBelow int 8 regular {axi_slave 0}  }
	{ rowCenter int 8 regular {axi_slave 0}  }
	{ rowAbove int 8 regular {axi_slave 0}  }
	{ outputRow int 8 regular {axi_slave 1}  }
}
set C_modelArgMapList {[ 
	{ "Name" : "rowBelow", "interface" : "axi_slave", "bundle":"AXI_CPU","type":"ap_memory","bitwidth" : 8, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "rowBelow","cData": "char","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}], "offset" : {"in":128}, "offset_end" : {"in":255}} , 
 	{ "Name" : "rowCenter", "interface" : "axi_slave", "bundle":"AXI_CPU","type":"ap_memory","bitwidth" : 8, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "rowCenter","cData": "char","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}], "offset" : {"in":256}, "offset_end" : {"in":383}} , 
 	{ "Name" : "rowAbove", "interface" : "axi_slave", "bundle":"AXI_CPU","type":"ap_memory","bitwidth" : 8, "direction" : "READONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "rowAbove","cData": "char","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}], "offset" : {"in":384}, "offset_end" : {"in":511}} , 
 	{ "Name" : "outputRow", "interface" : "axi_slave", "bundle":"AXI_CPU","type":"ap_memory","bitwidth" : 8, "direction" : "WRITEONLY", "bitSlice":[{"low":0,"up":0,"cElement": [{"cName": "outputRow","cData": "char","bit_use": { "low": 0,"up": 0},"cArray": [{"low" : 0,"up" : 0,"step" : 0}]}]}], "offset" : {"out":512}, "offset_end" : {"out":639}} ]}
# RTL Port declarations: 
set portNum 20
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst_n sc_in sc_logic 1 reset -1 active_low_sync } 
	{ s_axi_AXI_CPU_AWVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_AWREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_AWADDR sc_in sc_lv 10 signal -1 } 
	{ s_axi_AXI_CPU_WVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_WREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_WDATA sc_in sc_lv 32 signal -1 } 
	{ s_axi_AXI_CPU_WSTRB sc_in sc_lv 4 signal -1 } 
	{ s_axi_AXI_CPU_ARVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_ARREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_ARADDR sc_in sc_lv 10 signal -1 } 
	{ s_axi_AXI_CPU_RVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_RREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_RDATA sc_out sc_lv 32 signal -1 } 
	{ s_axi_AXI_CPU_RRESP sc_out sc_lv 2 signal -1 } 
	{ s_axi_AXI_CPU_BVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_BREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_AXI_CPU_BRESP sc_out sc_lv 2 signal -1 } 
	{ interrupt sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "s_axi_AXI_CPU_AWADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":10, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "AWADDR" },"address":[{"name":"filterImage","role":"start","value":"0","valid_bit":"0"},{"name":"filterImage","role":"continue","value":"0","valid_bit":"4"},{"name":"filterImage","role":"auto_start","value":"0","valid_bit":"7"},{"name":"rowBelow","role":"data","value":"128"},{"name":"rowCenter","role":"data","value":"256"},{"name":"rowAbove","role":"data","value":"384"}] },
	{ "name": "s_axi_AXI_CPU_AWVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "AWVALID" } },
	{ "name": "s_axi_AXI_CPU_AWREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "AWREADY" } },
	{ "name": "s_axi_AXI_CPU_WVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "WVALID" } },
	{ "name": "s_axi_AXI_CPU_WREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "WREADY" } },
	{ "name": "s_axi_AXI_CPU_WDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "WDATA" } },
	{ "name": "s_axi_AXI_CPU_WSTRB", "direction": "in", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "WSTRB" } },
	{ "name": "s_axi_AXI_CPU_ARADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":10, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "ARADDR" },"address":[{"name":"filterImage","role":"start","value":"0","valid_bit":"0"},{"name":"filterImage","role":"done","value":"0","valid_bit":"1"},{"name":"filterImage","role":"idle","value":"0","valid_bit":"2"},{"name":"filterImage","role":"ready","value":"0","valid_bit":"3"},{"name":"filterImage","role":"auto_start","value":"0","valid_bit":"7"},{"name":"outputRow","role":"data","value":"512"}] },
	{ "name": "s_axi_AXI_CPU_ARVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "ARVALID" } },
	{ "name": "s_axi_AXI_CPU_ARREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "ARREADY" } },
	{ "name": "s_axi_AXI_CPU_RVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "RVALID" } },
	{ "name": "s_axi_AXI_CPU_RREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "RREADY" } },
	{ "name": "s_axi_AXI_CPU_RDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "RDATA" } },
	{ "name": "s_axi_AXI_CPU_RRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "RRESP" } },
	{ "name": "s_axi_AXI_CPU_BVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "BVALID" } },
	{ "name": "s_axi_AXI_CPU_BREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "BREADY" } },
	{ "name": "s_axi_AXI_CPU_BRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "BRESP" } },
	{ "name": "interrupt", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "AXI_CPU", "role": "interrupt" } }, 
 	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst_n", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst_n", "role": "default" }}  ]}

set RtlHierarchyInfo {[
	{"ID" : "0", "Level" : "0", "Path" : "`AUTOTB_DUT_INST", "Parent" : "", "Child" : ["1"],
		"CDFG" : "filterImage",
		"Protocol" : "ap_ctrl_hs",
		"ControlExist" : "1", "ap_start" : "1", "ap_ready" : "1", "ap_done" : "1", "ap_continue" : "0", "ap_idle" : "1", "real_start" : "0",
		"Pipeline" : "None", "UnalignedPipeline" : "0", "RewindPipeline" : "0", "ProcessNetwork" : "0",
		"II" : "0",
		"VariableLatency" : "1", "ExactLatency" : "-1", "EstimateLatencyMin" : "354", "EstimateLatencyMax" : "354",
		"Combinational" : "0",
		"Datapath" : "0",
		"ClockEnable" : "0",
		"HasSubDataflow" : "0",
		"InDataflowNetwork" : "0",
		"HasNonBlockingOperation" : "0",
		"Port" : [
			{"Name" : "rowBelow", "Type" : "Memory", "Direction" : "I"},
			{"Name" : "rowCenter", "Type" : "Memory", "Direction" : "I"},
			{"Name" : "rowAbove", "Type" : "Memory", "Direction" : "I"},
			{"Name" : "outputRow", "Type" : "Memory", "Direction" : "O"}]},
	{"ID" : "1", "Level" : "1", "Path" : "`AUTOTB_DUT_INST.AXI_CPU_s_axi_U", "Parent" : "0"}]}


set ArgLastReadFirstWriteLatency {
	filterImage {
		rowBelow {Type I LastRead 353 FirstWrite -1}
		rowCenter {Type I LastRead 353 FirstWrite -1}
		rowAbove {Type I LastRead 353 FirstWrite -1}
		outputRow {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "354", "Max" : "354"}
	, {"Name" : "Interval", "Min" : "355", "Max" : "355"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
}

set busDeadlockParameterList { 
}

# RTL port scheduling information:
set fifoSchedulingInfoList { 
}

# RTL bus port read request latency information:
set busReadReqLatencyList { 
}

# RTL bus port write response latency information:
set busWriteResLatencyList { 
}

# RTL array port load latency information:
set memoryLoadLatencyList { 
}
