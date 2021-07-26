`include "vie_define.h"

module vie_wb_stage(
    clock,
    reset,

    msbus_i,
    ws_allowin,
    wsbus_o,
    debugbus_o
);
input                          clock;
input                          reset;
input  [`Vmsbus          -1:0] msbus_i;
output                         ws_allowin;
output [`Vwsbus          -1:0] wsbus_o;
output [`Vdebugbus       -1:0] debugbus_o;

wire        ms_valid = msbus_i[71:71];
wire [6 :0] ms_dest  = msbus_i[70:64];
wire [31:0] ms_pc    = msbus_i[63:32];
wire [31:0] ms_res   = msbus_i[31: 0];

wire        rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;

assign wsbus_o[37:37] = rf_we;
assign wsbus_o[36:32] = rf_waddr;
assign wsbus_o[31: 0] = rf_wdata;

wire [31:0] debug_wb_pc;
wire [ 3:0] debug_wb_rf_wen;
wire [ 4:0] debug_wb_rf_wnum;
wire [31:0] debug_wb_rf_wdata;

assign debugbus_o[72:41] = debug_wb_pc;
assign debugbus_o[40:37] = debug_wb_rf_wen;
assign debugbus_o[36:32] = debug_wb_rf_wnum;
assign debugbus_o[31: 0] = debug_wb_rf_wdata;


reg  [`Vmsbus    -1:0] msbus_r;
wire        ws_valid;
wire [6 :0] ws_dest;
wire [31:0] ws_pc;
wire [31:0] ws_res;

assign{ws_valid,
       ws_dest,
       ws_pc,
       ws_res
} = msbus_r;


//control signals
reg ws_valid_r;
wire ws_cango;

assign ws_cango = 1'b1;
assign ws_allowin = !ws_valid_r || ws_cango;

always @(posedge clock) begin
    if(reset) begin
        ws_valid_r <= 1'b0;
    end else if(ws_allowin) begin
        ws_valid_r <= ms_valid;
    end

    if(ms_valid && ws_allowin) begin
        msbus_r <= msbus_i;
    end
end
assign rf_we    = (ws_dest[6:5] == 2'b00) && ws_valid_r;
assign rf_waddr = ws_dest[4:0];
assign rf_wdata = ws_res;

assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_wen   = {4{rf_we}};
assign debug_wb_rf_wnum  = rf_waddr;
assign debug_wb_rf_wdata = rf_wdata;

endmodule