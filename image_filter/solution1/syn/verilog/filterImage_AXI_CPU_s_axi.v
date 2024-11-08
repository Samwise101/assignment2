// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
`timescale 1ns/1ps
module filterImage_AXI_CPU_s_axi
#(parameter
    C_S_AXI_ADDR_WIDTH = 10,
    C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                          ACLK,
    input  wire                          ARESET,
    input  wire                          ACLK_EN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] AWADDR,
    input  wire                          AWVALID,
    output wire                          AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0] WDATA,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0] WSTRB,
    input  wire                          WVALID,
    output wire                          WREADY,
    output wire [1:0]                    BRESP,
    output wire                          BVALID,
    input  wire                          BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] ARADDR,
    input  wire                          ARVALID,
    output wire                          ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0] RDATA,
    output wire [1:0]                    RRESP,
    output wire                          RVALID,
    input  wire                          RREADY,
    output wire                          interrupt,
    input  wire [6:0]                    rowBelow_address0,
    input  wire                          rowBelow_ce0,
    output wire [7:0]                    rowBelow_q0,
    input  wire [6:0]                    rowCenter_address0,
    input  wire                          rowCenter_ce0,
    output wire [7:0]                    rowCenter_q0,
    input  wire [6:0]                    rowAbove_address0,
    input  wire                          rowAbove_ce0,
    output wire [7:0]                    rowAbove_q0,
    input  wire [6:0]                    outputRow_address0,
    input  wire                          outputRow_ce0,
    input  wire                          outputRow_we0,
    input  wire [7:0]                    outputRow_d0,
    output wire                          ap_start,
    input  wire                          ap_done,
    input  wire                          ap_ready,
    input  wire                          ap_idle
);
//------------------------Address Info-------------------
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

//------------------------Parameter----------------------
localparam
    ADDR_AP_CTRL        = 10'h000,
    ADDR_GIE            = 10'h004,
    ADDR_IER            = 10'h008,
    ADDR_ISR            = 10'h00c,
    ADDR_ROWBELOW_BASE  = 10'h080,
    ADDR_ROWBELOW_HIGH  = 10'h0ff,
    ADDR_ROWCENTER_BASE = 10'h100,
    ADDR_ROWCENTER_HIGH = 10'h17f,
    ADDR_ROWABOVE_BASE  = 10'h180,
    ADDR_ROWABOVE_HIGH  = 10'h1ff,
    ADDR_OUTPUTROW_BASE = 10'h200,
    ADDR_OUTPUTROW_HIGH = 10'h27f,
    WRIDLE              = 2'd0,
    WRDATA              = 2'd1,
    WRRESP              = 2'd2,
    WRRESET             = 2'd3,
    RDIDLE              = 2'd0,
    RDDATA              = 2'd1,
    RDRESET             = 2'd2,
    ADDR_BITS                = 10;

//------------------------Local signal-------------------
    reg  [1:0]                    wstate = WRRESET;
    reg  [1:0]                    wnext;
    reg  [ADDR_BITS-1:0]          waddr;
    wire [C_S_AXI_DATA_WIDTH-1:0] wmask;
    wire                          aw_hs;
    wire                          w_hs;
    reg  [1:0]                    rstate = RDRESET;
    reg  [1:0]                    rnext;
    reg  [C_S_AXI_DATA_WIDTH-1:0] rdata;
    wire                          ar_hs;
    wire [ADDR_BITS-1:0]          raddr;
    // internal registers
    reg                           int_ap_idle;
    reg                           int_ap_ready;
    reg                           int_ap_done = 1'b0;
    reg                           int_ap_start = 1'b0;
    reg                           int_auto_restart = 1'b0;
    reg                           int_gie = 1'b0;
    reg  [1:0]                    int_ier = 2'b0;
    reg  [1:0]                    int_isr = 2'b0;
    // memory signals
    wire [4:0]                    int_rowBelow_address0;
    wire                          int_rowBelow_ce0;
    wire                          int_rowBelow_we0;
    wire [3:0]                    int_rowBelow_be0;
    wire [31:0]                   int_rowBelow_d0;
    wire [31:0]                   int_rowBelow_q0;
    wire [4:0]                    int_rowBelow_address1;
    wire                          int_rowBelow_ce1;
    wire                          int_rowBelow_we1;
    wire [3:0]                    int_rowBelow_be1;
    wire [31:0]                   int_rowBelow_d1;
    wire [31:0]                   int_rowBelow_q1;
    reg                           int_rowBelow_read;
    reg                           int_rowBelow_write;
    reg  [1:0]                    int_rowBelow_shift;
    wire [4:0]                    int_rowCenter_address0;
    wire                          int_rowCenter_ce0;
    wire                          int_rowCenter_we0;
    wire [3:0]                    int_rowCenter_be0;
    wire [31:0]                   int_rowCenter_d0;
    wire [31:0]                   int_rowCenter_q0;
    wire [4:0]                    int_rowCenter_address1;
    wire                          int_rowCenter_ce1;
    wire                          int_rowCenter_we1;
    wire [3:0]                    int_rowCenter_be1;
    wire [31:0]                   int_rowCenter_d1;
    wire [31:0]                   int_rowCenter_q1;
    reg                           int_rowCenter_read;
    reg                           int_rowCenter_write;
    reg  [1:0]                    int_rowCenter_shift;
    wire [4:0]                    int_rowAbove_address0;
    wire                          int_rowAbove_ce0;
    wire                          int_rowAbove_we0;
    wire [3:0]                    int_rowAbove_be0;
    wire [31:0]                   int_rowAbove_d0;
    wire [31:0]                   int_rowAbove_q0;
    wire [4:0]                    int_rowAbove_address1;
    wire                          int_rowAbove_ce1;
    wire                          int_rowAbove_we1;
    wire [3:0]                    int_rowAbove_be1;
    wire [31:0]                   int_rowAbove_d1;
    wire [31:0]                   int_rowAbove_q1;
    reg                           int_rowAbove_read;
    reg                           int_rowAbove_write;
    reg  [1:0]                    int_rowAbove_shift;
    wire [4:0]                    int_outputRow_address0;
    wire                          int_outputRow_ce0;
    wire                          int_outputRow_we0;
    wire [3:0]                    int_outputRow_be0;
    wire [31:0]                   int_outputRow_d0;
    wire [31:0]                   int_outputRow_q0;
    wire [4:0]                    int_outputRow_address1;
    wire                          int_outputRow_ce1;
    wire                          int_outputRow_we1;
    wire [3:0]                    int_outputRow_be1;
    wire [31:0]                   int_outputRow_d1;
    wire [31:0]                   int_outputRow_q1;
    reg                           int_outputRow_read;
    reg                           int_outputRow_write;
    reg  [1:0]                    int_outputRow_shift;

//------------------------Instantiation------------------
// int_rowBelow
filterImage_AXI_CPU_s_axi_ram #(
    .BYTES    ( 4 ),
    .DEPTH    ( 30 )
) int_rowBelow (
    .clk0     ( ACLK ),
    .address0 ( int_rowBelow_address0 ),
    .ce0      ( int_rowBelow_ce0 ),
    .we0      ( int_rowBelow_we0 ),
    .be0      ( int_rowBelow_be0 ),
    .d0       ( int_rowBelow_d0 ),
    .q0       ( int_rowBelow_q0 ),
    .clk1     ( ACLK ),
    .address1 ( int_rowBelow_address1 ),
    .ce1      ( int_rowBelow_ce1 ),
    .we1      ( int_rowBelow_we1 ),
    .be1      ( int_rowBelow_be1 ),
    .d1       ( int_rowBelow_d1 ),
    .q1       ( int_rowBelow_q1 )
);
// int_rowCenter
filterImage_AXI_CPU_s_axi_ram #(
    .BYTES    ( 4 ),
    .DEPTH    ( 30 )
) int_rowCenter (
    .clk0     ( ACLK ),
    .address0 ( int_rowCenter_address0 ),
    .ce0      ( int_rowCenter_ce0 ),
    .we0      ( int_rowCenter_we0 ),
    .be0      ( int_rowCenter_be0 ),
    .d0       ( int_rowCenter_d0 ),
    .q0       ( int_rowCenter_q0 ),
    .clk1     ( ACLK ),
    .address1 ( int_rowCenter_address1 ),
    .ce1      ( int_rowCenter_ce1 ),
    .we1      ( int_rowCenter_we1 ),
    .be1      ( int_rowCenter_be1 ),
    .d1       ( int_rowCenter_d1 ),
    .q1       ( int_rowCenter_q1 )
);
// int_rowAbove
filterImage_AXI_CPU_s_axi_ram #(
    .BYTES    ( 4 ),
    .DEPTH    ( 30 )
) int_rowAbove (
    .clk0     ( ACLK ),
    .address0 ( int_rowAbove_address0 ),
    .ce0      ( int_rowAbove_ce0 ),
    .we0      ( int_rowAbove_we0 ),
    .be0      ( int_rowAbove_be0 ),
    .d0       ( int_rowAbove_d0 ),
    .q0       ( int_rowAbove_q0 ),
    .clk1     ( ACLK ),
    .address1 ( int_rowAbove_address1 ),
    .ce1      ( int_rowAbove_ce1 ),
    .we1      ( int_rowAbove_we1 ),
    .be1      ( int_rowAbove_be1 ),
    .d1       ( int_rowAbove_d1 ),
    .q1       ( int_rowAbove_q1 )
);
// int_outputRow
filterImage_AXI_CPU_s_axi_ram #(
    .BYTES    ( 4 ),
    .DEPTH    ( 30 )
) int_outputRow (
    .clk0     ( ACLK ),
    .address0 ( int_outputRow_address0 ),
    .ce0      ( int_outputRow_ce0 ),
    .we0      ( int_outputRow_we0 ),
    .be0      ( int_outputRow_be0 ),
    .d0       ( int_outputRow_d0 ),
    .q0       ( int_outputRow_q0 ),
    .clk1     ( ACLK ),
    .address1 ( int_outputRow_address1 ),
    .ce1      ( int_outputRow_ce1 ),
    .we1      ( int_outputRow_we1 ),
    .be1      ( int_outputRow_be1 ),
    .d1       ( int_outputRow_d1 ),
    .q1       ( int_outputRow_q1 )
);


//------------------------AXI write fsm------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA) && (!ar_hs);
assign BRESP   = 2'b00;  // OKAY
assign BVALID  = (wstate == WRRESP);
assign wmask   = { {8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}} };
assign aw_hs   = AWVALID & AWREADY;
assign w_hs    = WVALID & WREADY;

// wstate
always @(posedge ACLK) begin
    if (ARESET)
        wstate <= WRRESET;
    else if (ACLK_EN)
        wstate <= wnext;
end

// wnext
always @(*) begin
    case (wstate)
        WRIDLE:
            if (AWVALID)
                wnext = WRDATA;
            else
                wnext = WRIDLE;
        WRDATA:
            if (w_hs)
                wnext = WRRESP;
            else
                wnext = WRDATA;
        WRRESP:
            if (BREADY)
                wnext = WRIDLE;
            else
                wnext = WRRESP;
        default:
            wnext = WRIDLE;
    endcase
end

// waddr
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (aw_hs)
            waddr <= AWADDR[ADDR_BITS-1:0];
    end
end

//------------------------AXI read fsm-------------------
assign ARREADY = (rstate == RDIDLE);
assign RDATA   = rdata;
assign RRESP   = 2'b00;  // OKAY
assign RVALID  = (rstate == RDDATA) & !int_rowBelow_read & !int_rowCenter_read & !int_rowAbove_read & !int_outputRow_read;
assign ar_hs   = ARVALID & ARREADY;
assign raddr   = ARADDR[ADDR_BITS-1:0];

// rstate
always @(posedge ACLK) begin
    if (ARESET)
        rstate <= RDRESET;
    else if (ACLK_EN)
        rstate <= rnext;
end

// rnext
always @(*) begin
    case (rstate)
        RDIDLE:
            if (ARVALID)
                rnext = RDDATA;
            else
                rnext = RDIDLE;
        RDDATA:
            if (RREADY & RVALID)
                rnext = RDIDLE;
            else
                rnext = RDDATA;
        default:
            rnext = RDIDLE;
    endcase
end

// rdata
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (ar_hs) begin
            rdata <= 'b0;
            case (raddr)
                ADDR_AP_CTRL: begin
                    rdata[0] <= int_ap_start;
                    rdata[1] <= int_ap_done;
                    rdata[2] <= int_ap_idle;
                    rdata[3] <= int_ap_ready;
                    rdata[7] <= int_auto_restart;
                end
                ADDR_GIE: begin
                    rdata <= int_gie;
                end
                ADDR_IER: begin
                    rdata <= int_ier;
                end
                ADDR_ISR: begin
                    rdata <= int_isr;
                end
            endcase
        end
        else if (int_rowBelow_read) begin
            rdata <= int_rowBelow_q1;
        end
        else if (int_rowCenter_read) begin
            rdata <= int_rowCenter_q1;
        end
        else if (int_rowAbove_read) begin
            rdata <= int_rowAbove_q1;
        end
        else if (int_outputRow_read) begin
            rdata <= int_outputRow_q1;
        end
    end
end


//------------------------Register logic-----------------
assign interrupt = int_gie & (|int_isr);
assign ap_start  = int_ap_start;
// int_ap_start
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_start <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0] && WDATA[0])
            int_ap_start <= 1'b1;
        else if (ap_ready)
            int_ap_start <= int_auto_restart; // clear on handshake/auto restart
    end
end

// int_ap_done
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_done <= 1'b0;
    else if (ACLK_EN) begin
        if (ap_done)
            int_ap_done <= 1'b1;
        else if (ar_hs && raddr == ADDR_AP_CTRL)
            int_ap_done <= 1'b0; // clear on read
    end
end

// int_ap_idle
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_idle <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_idle <= ap_idle;
    end
end

// int_ap_ready
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_ready <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_ready <= ap_ready;
    end
end

// int_auto_restart
always @(posedge ACLK) begin
    if (ARESET)
        int_auto_restart <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0])
            int_auto_restart <=  WDATA[7];
    end
end

// int_gie
always @(posedge ACLK) begin
    if (ARESET)
        int_gie <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_GIE && WSTRB[0])
            int_gie <= WDATA[0];
    end
end

// int_ier
always @(posedge ACLK) begin
    if (ARESET)
        int_ier <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_IER && WSTRB[0])
            int_ier <= WDATA[1:0];
    end
end

// int_isr[0]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[0] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[0] & ap_done)
            int_isr[0] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[0] <= int_isr[0] ^ WDATA[0]; // toggle on write
    end
end

// int_isr[1]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[1] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[1] & ap_ready)
            int_isr[1] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[1] <= int_isr[1] ^ WDATA[1]; // toggle on write
    end
end


//------------------------Memory logic-------------------
// rowBelow
assign int_rowBelow_address0  = rowBelow_address0 >> 2;
assign int_rowBelow_ce0       = rowBelow_ce0;
assign int_rowBelow_we0       = 1'b0;
assign int_rowBelow_be0       = 1'b0;
assign int_rowBelow_d0        = 1'b0;
assign rowBelow_q0            = int_rowBelow_q0 >> (int_rowBelow_shift * 8);
assign int_rowBelow_address1  = ar_hs? raddr[6:2] : waddr[6:2];
assign int_rowBelow_ce1       = ar_hs | (int_rowBelow_write & WVALID);
assign int_rowBelow_we1       = int_rowBelow_write & w_hs;
assign int_rowBelow_be1       = WSTRB;
assign int_rowBelow_d1        = WDATA;
// rowCenter
assign int_rowCenter_address0 = rowCenter_address0 >> 2;
assign int_rowCenter_ce0      = rowCenter_ce0;
assign int_rowCenter_we0      = 1'b0;
assign int_rowCenter_be0      = 1'b0;
assign int_rowCenter_d0       = 1'b0;
assign rowCenter_q0           = int_rowCenter_q0 >> (int_rowCenter_shift * 8);
assign int_rowCenter_address1 = ar_hs? raddr[6:2] : waddr[6:2];
assign int_rowCenter_ce1      = ar_hs | (int_rowCenter_write & WVALID);
assign int_rowCenter_we1      = int_rowCenter_write & w_hs;
assign int_rowCenter_be1      = WSTRB;
assign int_rowCenter_d1       = WDATA;
// rowAbove
assign int_rowAbove_address0  = rowAbove_address0 >> 2;
assign int_rowAbove_ce0       = rowAbove_ce0;
assign int_rowAbove_we0       = 1'b0;
assign int_rowAbove_be0       = 1'b0;
assign int_rowAbove_d0        = 1'b0;
assign rowAbove_q0            = int_rowAbove_q0 >> (int_rowAbove_shift * 8);
assign int_rowAbove_address1  = ar_hs? raddr[6:2] : waddr[6:2];
assign int_rowAbove_ce1       = ar_hs | (int_rowAbove_write & WVALID);
assign int_rowAbove_we1       = int_rowAbove_write & w_hs;
assign int_rowAbove_be1       = WSTRB;
assign int_rowAbove_d1        = WDATA;
// outputRow
assign int_outputRow_address0 = outputRow_address0 >> 2;
assign int_outputRow_ce0      = outputRow_ce0;
assign int_outputRow_we0      = outputRow_we0;
assign int_outputRow_be0      = 1 << outputRow_address0[1:0];
assign int_outputRow_d0       = {4{outputRow_d0}};
assign int_outputRow_address1 = ar_hs? raddr[6:2] : waddr[6:2];
assign int_outputRow_ce1      = ar_hs | (int_outputRow_write & WVALID);
assign int_outputRow_we1      = int_outputRow_write & w_hs;
assign int_outputRow_be1      = WSTRB;
assign int_outputRow_d1       = WDATA;
// int_rowBelow_read
always @(posedge ACLK) begin
    if (ARESET)
        int_rowBelow_read <= 1'b0;
    else if (ACLK_EN) begin
        if (ar_hs && raddr >= ADDR_ROWBELOW_BASE && raddr <= ADDR_ROWBELOW_HIGH)
            int_rowBelow_read <= 1'b1;
        else
            int_rowBelow_read <= 1'b0;
    end
end

// int_rowBelow_write
always @(posedge ACLK) begin
    if (ARESET)
        int_rowBelow_write <= 1'b0;
    else if (ACLK_EN) begin
        if (aw_hs && AWADDR[ADDR_BITS-1:0] >= ADDR_ROWBELOW_BASE && AWADDR[ADDR_BITS-1:0] <= ADDR_ROWBELOW_HIGH)
            int_rowBelow_write <= 1'b1;
        else if (w_hs)
            int_rowBelow_write <= 1'b0;
    end
end

// int_rowBelow_shift
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (rowBelow_ce0)
            int_rowBelow_shift <= rowBelow_address0[1:0];
    end
end

// int_rowCenter_read
always @(posedge ACLK) begin
    if (ARESET)
        int_rowCenter_read <= 1'b0;
    else if (ACLK_EN) begin
        if (ar_hs && raddr >= ADDR_ROWCENTER_BASE && raddr <= ADDR_ROWCENTER_HIGH)
            int_rowCenter_read <= 1'b1;
        else
            int_rowCenter_read <= 1'b0;
    end
end

// int_rowCenter_write
always @(posedge ACLK) begin
    if (ARESET)
        int_rowCenter_write <= 1'b0;
    else if (ACLK_EN) begin
        if (aw_hs && AWADDR[ADDR_BITS-1:0] >= ADDR_ROWCENTER_BASE && AWADDR[ADDR_BITS-1:0] <= ADDR_ROWCENTER_HIGH)
            int_rowCenter_write <= 1'b1;
        else if (w_hs)
            int_rowCenter_write <= 1'b0;
    end
end

// int_rowCenter_shift
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (rowCenter_ce0)
            int_rowCenter_shift <= rowCenter_address0[1:0];
    end
end

// int_rowAbove_read
always @(posedge ACLK) begin
    if (ARESET)
        int_rowAbove_read <= 1'b0;
    else if (ACLK_EN) begin
        if (ar_hs && raddr >= ADDR_ROWABOVE_BASE && raddr <= ADDR_ROWABOVE_HIGH)
            int_rowAbove_read <= 1'b1;
        else
            int_rowAbove_read <= 1'b0;
    end
end

// int_rowAbove_write
always @(posedge ACLK) begin
    if (ARESET)
        int_rowAbove_write <= 1'b0;
    else if (ACLK_EN) begin
        if (aw_hs && AWADDR[ADDR_BITS-1:0] >= ADDR_ROWABOVE_BASE && AWADDR[ADDR_BITS-1:0] <= ADDR_ROWABOVE_HIGH)
            int_rowAbove_write <= 1'b1;
        else if (w_hs)
            int_rowAbove_write <= 1'b0;
    end
end

// int_rowAbove_shift
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (rowAbove_ce0)
            int_rowAbove_shift <= rowAbove_address0[1:0];
    end
end

// int_outputRow_read
always @(posedge ACLK) begin
    if (ARESET)
        int_outputRow_read <= 1'b0;
    else if (ACLK_EN) begin
        if (ar_hs && raddr >= ADDR_OUTPUTROW_BASE && raddr <= ADDR_OUTPUTROW_HIGH)
            int_outputRow_read <= 1'b1;
        else
            int_outputRow_read <= 1'b0;
    end
end

// int_outputRow_write
always @(posedge ACLK) begin
    if (ARESET)
        int_outputRow_write <= 1'b0;
    else if (ACLK_EN) begin
        if (aw_hs && AWADDR[ADDR_BITS-1:0] >= ADDR_OUTPUTROW_BASE && AWADDR[ADDR_BITS-1:0] <= ADDR_OUTPUTROW_HIGH)
            int_outputRow_write <= 1'b1;
        else if (w_hs)
            int_outputRow_write <= 1'b0;
    end
end

// int_outputRow_shift
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (outputRow_ce0)
            int_outputRow_shift <= outputRow_address0[1:0];
    end
end


endmodule


`timescale 1ns/1ps

module filterImage_AXI_CPU_s_axi_ram
#(parameter
    BYTES  = 4,
    DEPTH  = 256,
    AWIDTH = log2(DEPTH)
) (
    input  wire               clk0,
    input  wire [AWIDTH-1:0]  address0,
    input  wire               ce0,
    input  wire               we0,
    input  wire [BYTES-1:0]   be0,
    input  wire [BYTES*8-1:0] d0,
    output reg  [BYTES*8-1:0] q0,
    input  wire               clk1,
    input  wire [AWIDTH-1:0]  address1,
    input  wire               ce1,
    input  wire               we1,
    input  wire [BYTES-1:0]   be1,
    input  wire [BYTES*8-1:0] d1,
    output reg  [BYTES*8-1:0] q1
);
//------------------------Local signal-------------------
reg  [BYTES*8-1:0] mem[0:DEPTH-1];
//------------------------Task and function--------------
function integer log2;
    input integer x;
    integer n, m;
begin
    n = 1;
    m = 2;
    while (m < x) begin
        n = n + 1;
        m = m * 2;
    end
    log2 = n;
end
endfunction
//------------------------Body---------------------------
// read port 0
always @(posedge clk0) begin
    if (ce0) q0 <= mem[address0];
end

// read port 1
always @(posedge clk1) begin
    if (ce1) q1 <= mem[address1];
end

genvar i;
generate
    for (i = 0; i < BYTES; i = i + 1) begin : gen_write
        // write port 0
        always @(posedge clk0) begin
            if (ce0 & we0 & be0[i]) begin
                mem[address0][8*i+7:8*i] <= d0[8*i+7:8*i];
            end
        end
        // write port 1
        always @(posedge clk1) begin
            if (ce1 & we1 & be1[i]) begin
                mem[address1][8*i+7:8*i] <= d1[8*i+7:8*i];
            end
        end
    end
endgenerate

endmodule

