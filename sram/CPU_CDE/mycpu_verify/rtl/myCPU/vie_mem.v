module vie_mem_stage(
    clock,
    reset,
    
    rsbus_i,
    ifc_data_i,
    ws_allowin,
    msbus_o,
    mstatus_o,
    ms_allowin,
    flushbus_i,
    wstate_i,
    mstate_o
);
input                        clock;
input                        reset;

input  [`Vrsbus        -1:0] rsbus_i;
input  [`Vfromifcbus   -1:0] ifc_data_i;
input                        ws_allowin;
input  [`Vflushbus     -1:0] flushbus_i;
input  [`Vwstate       -1:0] wstate_i  ;
output [`Vmstate       -1:0] mstate_o  ;
output [`Vmsbus        -1:0] msbus_o;
output                       ms_allowin; 
output [`Vmstatus      -1:0] mstatus_o;



wire        rs_valid        = rsbus_i[97:97];
wire        rs_bd           = rsbus_i[96:96];
wire [7 :9] rs_cp0_addr     = rsbus_i[95:88];
wire        rs_exc          = rsbus_i[87:82];
wire        rs_res_from_mem = rsbus_i[81:81];
wire [7 :0] rs_op           = rsbus_i[80:73];
wire [1 :0] rs_sel          = rsbus_i[72:71];
wire [6 :0] rs_dest         = rsbus_i[70:64];
wire [31:0] rs_fixres       = rsbus_i[63:32];
wire [31:0] rs_pc           = rsbus_i[31: 0];

wire [31:0] mem_data        = ifc_data_i;


wire        flush_taken  = flushbus_i[32:32];
wire [31:0] flush_target = flushbus_i[31: 0];

wire        final_valid   ;
wire        final_bd      ;
wire [7 :0] final_op      ;
wire [7 :0] final_cp0_addr;
wire [5 :0] final_exc     ;
wire [6 :0] final_dest    ;
wire [31:0] final_pc      ;
wire [31:0] final_res     ;

assign msbus_o[94:94] = final_valid;
assign msbus_o[93:93] = final_bd   ;
assign msbus_o[92:85] = final_op   ;
assign msbus_o[84:77] = final_cp0_addr;
assign msbus_o[76:71] = final_exc  ;
assign msbus_o[70:64] = final_dest ;
assign msbus_o[63:32] = final_pc   ;
assign msbus_o[31: 0] = final_res  ;


wire   state_kernel   = wstate_i[1:1];
wire   state_exc      = wstate_i[0:0];


wire    cur_kernel;
wire    cur_exc;

assign mstate_o[1:1]  = cur_kernel;
assign mstate_o[0:0]  = cur_exc;



reg [`Vrsbus - 1:0] rsbus_r;

wire        ms_valid;
wire        ms_bd;
wire [7 :0] ms_cp0_addr;
wire [5 :0] ms_exc;
wire        ms_res_from_mem;
wire [7 :0] ms_op;
wire [1 :0] ms_sel;
wire [6 :0] ms_dest;
wire [31:0] ms_fixres;
wire [31:0] ms_pc;

assign {ms_valid,
        ms_bd,
        ms_cp0_addr,
        ms_exc,
        ms_res_from_mem,
        ms_op,
        ms_sel,
        ms_dest,
        ms_fixres,
        ms_pc
} = rsbus_r;

//exc
wire exc;
// wire [31:0] data;

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
        if(flush_taken) ms_valid_r <= 1'b0;
        else ms_valid_r <= rs_valid;
    end

    if(rs_valid && ms_allowin) begin
        rsbus_r <= rsbus_i;
    end
end

//load
wire op_lw  = ms_op == `VIE_OP_LW;

wire op_lb  = ms_op == `VIE_OP_LB;

wire op_lbu = ms_op == `VIE_OP_LBU;

wire op_lh  = ms_op == `VIE_OP_LH;

wire op_lhu = ms_op == `VIE_OP_LHU;

wire op_lwl = ms_op == `VIE_OP_LWL;

wire op_lwr = ms_op == `VIE_OP_LWR;

wire [31:0] load_res;
wire [31:0] lw_res;
wire [31:0] lb_res;
wire [31:0] lbu_res;
wire [31:0] lh_res;
wire [31:0] lhu_res;
wire [31:0] lwl_res;
wire [31:0] lwr_res;

assign lw_res   =  {32{op_lw}}  & mem_data;

assign lb_res   =  {32{op_lb}} & ({32{(ms_sel[1:0] == 2'b00)}}  & {{24{mem_data[7]}} ,mem_data[7:0]  } |
                                  {32{(ms_sel[1:0] == 2'b01)}}  & {{24{mem_data[15]}},mem_data[15:8] } |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {{24{mem_data[23]}},mem_data[23:16]} |
                                  {32{(ms_sel[1:0] == 2'b11)}}  & {{24{mem_data[31]}},mem_data[31:24]});

assign lbu_res  = {32{op_lbu}} & ({32{(ms_sel[1:0] == 2'b00)}}  & {24'b0,mem_data[7:0]  } |
                                  {32{(ms_sel[1:0] == 2'b01)}}  & {24'b0,mem_data[15:8] } |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {24'b0,mem_data[23:16]} |
                                  {32{(ms_sel[1:0] == 2'b11)}}  & {24'b0,mem_data[31:24]});

assign lh_res   = {32{op_lh}}  & ({32{(ms_sel[1:0] == 2'b00)}}  & {{16{mem_data[15]}},mem_data[15:0]}  |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {{16{mem_data[31]}},mem_data[31:16]});

assign lhu_res  = {32{op_lhu}} & ({32{(ms_sel[1:0] == 2'b00)}}  & {16'b0,mem_data[15:0]} |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {16'b0,mem_data[31:16]});

assign lwl_res  = {32{op_lwl}} & ({32{(ms_sel[1:0] == 2'b00)}}  & {mem_data[7 : 0],ms_fixres[23:0]}  | 
                                  {32{(ms_sel[1:0] == 2'b01)}}  & {mem_data[15: 0],ms_fixres[15:0]}  |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {mem_data[23: 0],ms_fixres[ 7:0]}  |
                                  {32{(ms_sel[1:0] == 2'b11)}}  & mem_data[31:0]);

assign lwr_res  = {32{op_lwr}} & ({32{(ms_sel[1:0] == 2'b00)}}  & mem_data[31:0]                     |
                                  {32{(ms_sel[1:0] == 2'b01)}}  & {ms_fixres[31:24],mem_data[31: 8]} |
                                  {32{(ms_sel[1:0] == 2'b10)}}  & {ms_fixres[31:16],mem_data[31:16]} |
                                  {32{(ms_sel[1:0] == 2'b11)}}  & {ms_fixres[31: 8],mem_data[31:24]});

assign load_res = lw_res | lb_res | lbu_res | lh_res | lhu_res | lwl_res | lwr_res;


//exc
assign exc = ms_valid_r & (ms_exc[5] | ms_exc[4] | ms_exc[3] | ms_exc[2] | ms_exc[1] | ms_exc[0] | state_exc);
assign final_valid    = ms_to_ws_valid;
assign final_bd       = ms_bd         ;
assign final_op       = ms_op         ;
assign final_cp0_addr = ms_cp0_addr   ;
assign final_exc      = ms_exc        ;
assign final_dest     = ms_dest       ;
assign final_pc       = ms_pc         ;
assign final_res      = ms_res_from_mem ? load_res : ms_fixres;

assign mstatus_o[40:40] = ms_valid_r;
assign mstatus_o[39:39] = ms_op == `VIE_OP_MFC0;
assign mstatus_o[38:32] = final_dest;
assign mstatus_o[31: 0] = final_res;

assign cur_kernel       = state_kernel;
assign cur_exc          = exc;
endmodule