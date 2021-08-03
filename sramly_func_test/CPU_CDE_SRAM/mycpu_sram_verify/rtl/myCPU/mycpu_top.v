`include "vie_define.h"
module mycpu_top(
    input         clk,
    input         resetn,
//    input  [ 5:0] ext_int,
    // inst sram interface
    output        inst_sram_req,
    output        inst_sram_wr,
    output [ 1:0] inst_sram_size,
    output [ 3:0] inst_sram_wstrb,
    output [31:0] inst_sram_addr,
    output [31:0] inst_sram_wdata,
    input         inst_sram_addr_ok,
    input         inst_sram_data_ok,
    input  [31:0] inst_sram_rdata,
    // data sram interface
    output        data_sram_req,
    output        data_sram_wr,
    output [ 1:0] data_sram_size,
    output [ 3:0] data_sram_wstrb,
    output [31:0] data_sram_addr,
    output [31:0] data_sram_wdata,
    input         data_sram_addr_ok,
    input         data_sram_data_ok,
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
wire [`Vfromifcbus  -1:0] ifc_inst;     //取到的指�????
wire [`Vtoifcbus    -1:0] inst_ifc;     //
wire [`Vfromifcbus  -1:0] ifc_data;
wire [`Vtoifcbus    -1:0] data_ifc;
wire [`Vissuebus    -1:0] issuebus;
wire [`Vwsbus       -1:0] wsbus   ;
wire [`Vrsbus       -1:0] rsbus   ;
wire [`Vmsbus       -1:0] msbus   ;
wire [`Vdebugbus    -1:0] debugbus;
wire [`Vflushbus    -1:0] flushbus;
wire [`Vrstatus     -1:0] rstatus ;
wire [`Vmstatus     -1:0] mstatus ;
wire [`Vwstate      -1:0] wstate  ;
wire [`Vmstate      -1:0] mstate  ;
wire [`Vestate      -1:0] estate  ;

wire [5:0] ext_int         = 6'b000000;
assign  inst_sram_req   = inst_ifc[71:71];
assign  inst_sram_wr    = inst_ifc[70:70];
assign  inst_sram_size  = inst_ifc[69:68];
assign  inst_sram_wstrb = inst_ifc[67:64];
assign  inst_sram_addr  = inst_ifc[63:32];
assign  inst_sram_wdata = inst_ifc[31: 0];

assign  ifc_inst[33:33] = inst_sram_addr_ok;
assign  ifc_inst[32:32] = inst_sram_data_ok;
assign  ifc_inst[31: 0] = inst_sram_rdata  ;



assign  data_sram_req   = data_ifc[71:71];
assign  data_sram_wr    = data_ifc[70:70];
assign  data_sram_size  = data_ifc[69:68];
assign  data_sram_wstrb = data_ifc[67:64];
assign  data_sram_addr  = data_ifc[63:32];
assign  data_sram_wdata = data_ifc[31: 0];

assign  ifc_data[33:33]      = data_sram_addr_ok;
assign  ifc_data[32:32]      = data_sram_data_ok;
assign  ifc_data[31: 0]      = data_sram_rdata  ;

assign debug_wb_pc       = debugbus[72:41];
assign debug_wb_rf_wen   = debugbus[40:37];
assign debug_wb_rf_wnum  = debugbus[36:32];
assign debug_wb_rf_wdata = debugbus[31: 0];



vie_if_stage  u0_if_stage(
    .clock(clk)    , .reset(reset)        , .ds_allowin(ds_allowin), .brbus_i(brbus),
    .fsbus_o(fsbus), .ifc_inst_i(ifc_inst), .inst_ifc_o(inst_ifc)  , .flushbus_i(flushbus)
);

vie_id_stage  u1_id_stage(
    .clock(clk)    , .reset(reset)        , .es_allowin(es_allowin), .fsbus_i(fsbus),
    .brbus_o(brbus), .issuebus_o(issuebus), .ds_allowin(ds_allowin), .wsbus_i(wsbus),
    .rstatus_i(rstatus), .mstatus_i(mstatus), .flushbus_i(flushbus), .estate_i(estate)
);

vie_exe_stage u2_ex_stage(
    .clock(clk)          , .reset(reset)        , .es_allowin(es_allowin), .issuebus_i(issuebus),
    .rsbus_o(rsbus)      , .data_ifc_o(data_ifc), .ms_allowin(ms_allowin), .rstatus_o(rstatus) ,
    .flushbus_i(flushbus), .estate_o  (estate)  , .mstate_i(mstate)      , .ifc_data_i(ifc_data)
);

vie_mem_stage u3_mem_stage(
    .clock(clk)          , .reset(reset)          , .ws_allowin(ws_allowin), .rsbus_i(rsbus),
    .msbus_o(msbus)      , .ms_allowin(ms_allowin), .mstatus_o(mstatus),
    .flushbus_i(flushbus), .wstate_i(wstate)      , .mstate_o(mstate)
);

vie_wb_stage  u4_wb_stage(
    .clock(clk)    , .reset(reset)        , .ws_allowin(ws_allowin), .msbus_i(msbus)      ,
    .wsbus_o(wsbus), .debugbus_o(debugbus), .ext_int_in(ext_int)   , .flushbus_o(flushbus),
    .wstate_o(wstate)
);
endmodule
