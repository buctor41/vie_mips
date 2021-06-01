`include "vie_define.h"
module mycpu_top(
    input         clk,
    input         resetn,
    // inst sram interface
    output        inst_sram_en,
    output [ 3:0] inst_sram_wen,
    output [31:0] inst_sram_addr,
    output [31:0] inst_sram_wdata,
    input  [31:0] inst_sram_rdata,
    // data sram interface
    output        data_sram_en,
    output [ 3:0] data_sram_wen,
    output [31:0] data_sram_addr,
    output [31:0] data_sram_wdata,
    input  [31:0] data_sram_rdata,
    // trace debug interface
    output [31:0] debug_wb_pc,
    output [ 3:0] debug_wb_rf_wen,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

wire         ds_allowin;
wire         es_allowin;
wire         ms_allowin;
wire         ws_allowin;

wire [`Vbrbus       -1:0] brbus;
wire [`Vfsbus       -1:0] fsbus;
wire [`Vfromifcbus  -1:0] ifc_inst;     //取到的指�??
wire [`Vtoifcbus    -1:0] inst_ifc;     //
wire [`Vfromifcbus  -1:0] ifc_data;
wire [`Vtoifcbus    -1:0] data_ifc;
wire [`Vissuebus    -1:0] issuebus;
wire [`Vwsbus       -1:0] wsbus   ;
wire [`Vrsbus       -1:0] rsbus   ;
wire [`Vmsbus       -1:0] msbus   ;
wire [`Vdebugbus    -1:0] debugbus;
wire [`Vrstatus     -1:0] rstatus ;
wire [`Vmstatus     -1:0] mstatus ;

assign        ifc_inst = inst_sram_rdata;
assign    inst_sram_en = inst_ifc[68:68];
assign   inst_sram_wen = inst_ifc[67:64];
assign  inst_sram_addr = inst_ifc[63:32];
assign inst_sram_wdata = inst_ifc[31: 0];

assign        ifc_data = data_sram_rdata;
assign    data_sram_en = data_ifc[68:68];
assign   data_sram_wen = data_ifc[67:64];
assign  data_sram_addr = data_ifc[63:32];
assign data_sram_wdata = data_ifc[31: 0];

assign debug_wb_pc       = debugbus[72:41];
assign debug_wb_rf_wen   = debugbus[40:37];
assign debug_wb_rf_wnum  = debugbus[36:32];
assign debug_wb_rf_wdata = debugbus[31: 0];



vie_if_stage  u0_if_stage(
    .clock(clk)    , .reset(reset)        , .ds_allowin(ds_allowin), .brbus_i(brbus),
    .fsbus_o(fsbus), .ifc_inst_i(ifc_inst), .inst_ifc_o(inst_ifc)
);

vie_id_stage  u1_id_stage(
    .clock(clk)    , .reset(reset)        , .es_allowin(es_allowin), .fsbus_i(fsbus),
    .brbus_o(brbus), .issuebus_o(issuebus), .ds_allowin(ds_allowin), .wsbus_i(wsbus),
    .rstatus_i(rstatus), .mstatus_i(mstatus)
);

vie_exe_stage u2_ex_stage(
    .clock(clk)    , .reset(reset)        , .es_allowin(es_allowin), .issuebus_i(issuebus),
    .rsbus_o(rsbus), .data_ifc_o(data_ifc), .ms_allowin(ms_allowin), .rstatus_o(rstatus)
);

vie_mem_stage u3_mem_stage(
    .clock(clk)    , .reset(reset)        , .ws_allowin(ws_allowin), .rsbus_i(rsbus),
    .msbus_o(msbus), .ifc_data_i(ifc_data), .ms_allowin(ms_allowin), .mstatus_o(mstatus)
);

vie_wb_stage  u4_wb_stage(
    .clock(clk)    , .reset(reset)        , .ws_allowin(ws_allowin), .msbus_i(msbus),
    .wsbus_o(wsbus), .debugbus_o(debugbus)
);
endmodule
