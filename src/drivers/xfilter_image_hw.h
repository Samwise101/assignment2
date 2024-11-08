// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// AXI_CPU
// 0x000 : Control signals
//         bit 0  - ap_start (Read/Write/COH)
//         bit 1  - ap_done (Read/COR)
//         bit 2  - ap_idle (Read)
//         bit 3  - ap_ready (Read)
//         bit 7  - auto_restart (Read/Write)
//         others - reserved
// 0x004 : Global Interrupt Enable Register
//         bit 0  - Global Interrupt Enable (Read/Write)
//         others - reserved
// 0x008 : IP Interrupt Enable Register (Read/Write)
//         bit 0  - enable ap_done interrupt (Read/Write)
//         bit 1  - enable ap_ready interrupt (Read/Write)
//         others - reserved
// 0x00c : IP Interrupt Status Register (Read/TOW)
//         bit 0  - ap_done (COR/TOW)
//         bit 1  - ap_ready (COR/TOW)
//         others - reserved
// 0x080 ~
// 0x0ff : Memory 'rowBelow' (120 * 8b)
//         Word n : bit [ 7: 0] - rowBelow[4n]
//                  bit [15: 8] - rowBelow[4n+1]
//                  bit [23:16] - rowBelow[4n+2]
//                  bit [31:24] - rowBelow[4n+3]
// 0x100 ~
// 0x17f : Memory 'rowCenter' (120 * 8b)
//         Word n : bit [ 7: 0] - rowCenter[4n]
//                  bit [15: 8] - rowCenter[4n+1]
//                  bit [23:16] - rowCenter[4n+2]
//                  bit [31:24] - rowCenter[4n+3]
// 0x180 ~
// 0x1ff : Memory 'rowAbove' (120 * 8b)
//         Word n : bit [ 7: 0] - rowAbove[4n]
//                  bit [15: 8] - rowAbove[4n+1]
//                  bit [23:16] - rowAbove[4n+2]
//                  bit [31:24] - rowAbove[4n+3]
// 0x200 ~
// 0x27f : Memory 'outputRow' (120 * 8b)
//         Word n : bit [ 7: 0] - outputRow[4n]
//                  bit [15: 8] - outputRow[4n+1]
//                  bit [23:16] - outputRow[4n+2]
//                  bit [31:24] - outputRow[4n+3]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XFILTER_IMAGE_AXI_CPU_ADDR_AP_CTRL        0x000
#define XFILTER_IMAGE_AXI_CPU_ADDR_GIE            0x004
#define XFILTER_IMAGE_AXI_CPU_ADDR_IER            0x008
#define XFILTER_IMAGE_AXI_CPU_ADDR_ISR            0x00c
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_BASE  0x080
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWBELOW_HIGH  0x0ff
#define XFILTER_IMAGE_AXI_CPU_WIDTH_ROWBELOW      8
#define XFILTER_IMAGE_AXI_CPU_DEPTH_ROWBELOW      120
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_BASE 0x100
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWCENTER_HIGH 0x17f
#define XFILTER_IMAGE_AXI_CPU_WIDTH_ROWCENTER     8
#define XFILTER_IMAGE_AXI_CPU_DEPTH_ROWCENTER     120
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_BASE  0x180
#define XFILTER_IMAGE_AXI_CPU_ADDR_ROWABOVE_HIGH  0x1ff
#define XFILTER_IMAGE_AXI_CPU_WIDTH_ROWABOVE      8
#define XFILTER_IMAGE_AXI_CPU_DEPTH_ROWABOVE      120
#define XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_BASE 0x200
#define XFILTER_IMAGE_AXI_CPU_ADDR_OUTPUTROW_HIGH 0x27f
#define XFILTER_IMAGE_AXI_CPU_WIDTH_OUTPUTROW     8
#define XFILTER_IMAGE_AXI_CPU_DEPTH_OUTPUTROW     120

