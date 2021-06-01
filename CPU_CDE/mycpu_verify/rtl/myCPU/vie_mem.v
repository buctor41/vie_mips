`include "vie_define.h"
module vie_mem_stage(
    clock,
    reset,
    
    rsbus_i,
    ifc_data_i,
    ws_allowin,
    msbus_o,
    mstatus_o,
    ms_allowin
);
input                        clock;
input                        reset;

input  [`Vrsbus        -1:0] rsbus_i;
input  [`Vfromifcbus   -1:0] ifc_data_i;
input                        ws_allowin;
output [`Vmsbus        -1:0] msbus_o;
output                       ms_allowin; 
output [`Vmstatus      -1:0] mstatus_o;



wire        rs_valid        = rsbus_i[72:72];
wire        rs_res_from_mem = rsbus_i[71:71];
wire [6 :0] rs_dest         = rsbus_i[70:64];
wire [31:0] rs_fixres       = rsbus_i[63:32];
wire [31:0] rs_pc           = rsbus_i[31: 0];

wire [31:0] mem_data        = ifc_data_i;

wire        final_valid;
wire [6 :0] final_dest ;
wire [31:0] final_pc   ;
wire [31:0] final_res  ;

assign msbus_o[71:71] = final_valid;
assign msbus_o[70:64] = final_dest ;
assign msbus_o[63:32] = final_pc   ;
assign msbus_o[31: 0] = final_res  ;


reg [`Vrsbus - 1:0] rsbus_r;

wire        ms_valid;
wire        ms_res_from_mem;
wire [6:0]  ms_dest;
wire [31:0] ms_fixres;
wire [31:0] ms_pc;

assign {ms_valid,
        ms_res_from_mem,
        ms_dest,
        ms_fixres,
        ms_pc
} = rsbus_r;

wire [31:0] data;

//control signals
reg  ms_valid_r;
wire ms_cango;
wire ms_to_ws_valid;

assign ms_cango = 1'b1;
assign ms_allowin = !ms_valid_r || ms_cango && ws_allowin;
assign ms_to_ws_valid = ms_valid_r && ms_cango;

always @(posedge clock) begin
    if(reset) begin
        ms_valid_r <= 1'b0;
    end else if(ms_allowin) begin
        ms_valid_r <= rs_valid;
    end

    if(rs_valid && ms_allowin) begin
        rsbus_r <= rsbus_i;
    end
end

assign final_valid  = ms_to_ws_valid;
assign final_dest   = ms_dest       ;
assign final_pc     = ms_pc         ;
assign final_res    = ms_res_from_mem ? mem_data : ms_fixres;

assign mstatus_o[39:39] = ms_valid_r;
assign mstatus_o[38:32] = final_dest;
assign mstatus_o[31: 0] = final_res;
endmodule