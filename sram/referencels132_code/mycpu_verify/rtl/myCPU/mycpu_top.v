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
wire         fs_to_ds_valid;
wire         ds_to_es_valid;
wire         es_to_ms_valid;
wire         ms_to_ws_valid;
wire [`FS_TO_DS_BUS_WD    -1:0] fs_to_ds_bus;
wire [`DS_TO_ES_BUS_WD    -1:0] ds_to_es_bus;
wire [`ES_TO_MS_BUS_WD    -1:0] es_to_ms_bus;
wire [`MS_TO_WS_BUS_WD    -1:0] ms_to_ws_bus;
wire [`WS_TO_RF_BUS_WD    -1:0] ws_to_rf_bus;
wire [`BR_BUS_WD          -1:0] br_bus;
wire [`DATA_HAZARD_BUS_WD -1:0] data_hazard_bus;
wire [`ES_TO_DS_BUS_WD    -1:0] es_to_ds_bus;
wire [`MS_TO_DS_BUS_WD    -1:0] ms_to_ds_bus;
wire [`WS_TO_DS_BUS_WD    -1:0] ws_to_ds_bus;

assign data_hazard_bus = {es_to_ds_bus ,
                          ms_to_ds_bus ,
                          ws_to_ds_bus 
                         };


wire [`Vfsbus             -1:0] fsbus;
assign {fs_to_ds_valid,fs_to_ds_bus} = fsbus;
wire [`Vfromifcbus        -1:0] ifc_inst;
assign ifc_inst = inst_sram_rdata;
wire [`Vtoifcbus          -1:0] inst_ifc;
assign {inst_sram_en  ,
        inst_sram_wen ,
        inst_sram_addr,
        inst_sram_wdata
} = inst_ifc;
// IF stage
// if_stage if_stage(
//     .clk            (clk            ),
//     .reset          (reset          ),
//     //allowin
//     .ds_allowin     (ds_allowin     ),
//     //brbus
//     .br_bus         (br_bus         ),
//     //outputs
//     .fs_to_ds_valid (fs_to_ds_valid ),
//     .fs_to_ds_bus   (fs_to_ds_bus   ),
//     // inst sram interface
//     .inst_sram_en   (inst_sram_en   ),
//     .inst_sram_wen  (inst_sram_wen  ),
//     .inst_sram_addr (inst_sram_addr ),
//     .inst_sram_wdata(inst_sram_wdata),
//     .inst_sram_rdata(inst_sram_rdata)
// );
vie_if_stage if_stage(
    .clock          (clk            ),
    .reset          (reset          ),
    //ds_allowin
    .ds_allowin     (ds_allowin     ),
    //brbus 
    .brbus_i        (br_bus         ),
    .fsbus_o        (fsbus          ),

    .ifc_inst_i     (ifc_inst       ),
    .inst_ifc_o     (inst_ifc       )
);
// ID stage
id_stage id_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .es_allowin     (es_allowin     ),
    .ds_allowin     (ds_allowin     ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs
    .br_bus         (br_bus         ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //from es,ms,ws:for data hazard
    .data_hazard_bus(data_hazard_bus)
);
// EXE stage
exe_stage exe_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ms_allowin     (ms_allowin     ),
    .es_allowin     (es_allowin     ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ds:forewarding data that may cause data hazard
    .es_to_ds_bus   (es_to_ds_bus   ),
    // data sram interface
    .data_sram_en   (data_sram_en   ),
    .data_sram_wen  (data_sram_wen  ),
    .data_sram_addr (data_sram_addr ),
    .data_sram_wdata(data_sram_wdata)
);
// MEM stage
mem_stage mem_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    .ms_allowin     (ms_allowin     ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to ds:forewarding data that may cause data hazard
    .ms_to_ds_bus   (ms_to_ds_bus   ),    
    //from data-sram
    .data_sram_rdata(data_sram_rdata)
);
// WB stage
wb_stage wb_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //to ds:forewarding data that may cause data hazard
    .ws_to_ds_bus   (ws_to_ds_bus   ),
    //trace debug interface
    .debug_wb_pc      (debug_wb_pc      ),
    .debug_wb_rf_wen  (debug_wb_rf_wen  ),
    .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(debug_wb_rf_wdata)
);

endmodule
