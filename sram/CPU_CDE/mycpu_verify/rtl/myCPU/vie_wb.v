module vie_wb_stage(
    clock,
    reset,
    ext_int_in,
    msbus_i,
    ws_allowin,
    wsbus_o,
    debugbus_o,
    flushbus_o,
    wstate_o
);
input                          clock;
input                          reset;
input  [6                -1:0] ext_int_in;
input  [`Vmsbus          -1:0] msbus_i   ;
output                         ws_allowin;
output [`Vwsbus          -1:0] wsbus_o   ;
output [`Vdebugbus       -1:0] debugbus_o;
output [`Vflushbus       -1:0] flushbus_o;
output [`Vwstate         -1:0] wstate_o  ; 

wire        ms_valid    = msbus_i[94:94];
wire        ms_bd       = msbus_i[93:93];
wire [7 :0] ms_op       = msbus_i[92:85];
wire [7 :0] ms_cp0_addr = msbus_i[84:77];
wire [5 :0] ms_exc      = msbus_i[76:71];
wire [6 :0] ms_dest     = msbus_i[70:64];
wire [31:0] ms_pc       = msbus_i[63:32];
wire [31:0] ms_res      = msbus_i[31: 0];

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

wire        flush_happen;
wire [31:0] flush_target;

assign flushbus_o[32:32] = flush_happen;
assign flushbus_o[31: 0] = flush_target;



wire        state_kernel;
wire        state_exc;

assign wstate_o[1:1]     = state_kernel;
assign wstate_o[0:0]     = state_exc;

reg  [`Vmsbus    -1:0] msbus_r;
wire        ws_valid;
wire        ws_bd   ;
wire [7 :0] ws_op   ;
wire [7 :0] ws_cp0_addr;
wire [5 :0] ws_exc;
wire [6 :0] ws_dest;
wire [31:0] ws_pc;
wire [31:0] ws_res;

assign{ws_valid,
       ws_bd,
       ws_op,
       ws_cp0_addr,
       ws_exc,
       ws_dest,
       ws_pc,
       ws_res
} = msbus_r;


//Internal Signals
reg res_src;



//Cp0 Signals
wire op_mtc0 = ms_op == `VIE_OP_MTC0;
wire op_mfc0 = ms_op == `VIE_OP_MFC0;
wire bd;
wire cp0_ren;
wire cp0_wen;
wire [7 :0] cp0_addr;
wire [7 :0] cp0_waddr;
wire [7 :0] cp0_raddr;
wire [31:0] cp0_wr_value;
wire [31:0] cp0_rd_value;

//Cp0 Register

//Count         reg: 9, sel: 0
reg  [31:0]  c0_count;

wire [31:0]  count_value;
assign count_value = c0_count;

//Compare      reg: 11, sel:0
reg  [31:0]  c0_compare;

wire [31:0]  compare_value;
assign compare_value = c0_compare;
reg tick;

//Status        reg: 8, sel: 0
reg          c0_status_bev;
reg  [7:0]   c0_status_im ;
reg          c0_status_exl;
reg          c0_status_ie ;

wire [31:0]  status_value ;
assign status_value = {9'b0,c0_status_bev,6'b0,c0_status_im,
                       6'b0,c0_status_exl,c0_status_ie};

//Cause         reg: 13, sel: 0
reg           c0_cause_bd;
reg           c0_cause_ti;
reg  [7:0]    c0_cause_ip;
reg  [4:0]    c0_cause_excode;

wire [31:0]   cause_value;
assign cause_value  = {c0_cause_bd, c0_cause_ti    , 15'b0,c0_cause_ip,
                       1'b0       , c0_cause_excode, 2'b0};

//EPC           reg: 14, sel: 0
reg  [31:0]    c0_epc;

wire [31:0]     epc_value;
assign epc_value = c0_epc;




//Exception
reg  exc_r;
reg  exl_r;

wire exc;
wire exc_adel;
wire exc_ades;
wire exc_ov ; //INT OVERFLOW
wire exc_sys ;
wire exc_bp  ;
wire exc_ri  ; //SAVED_INSTRUCTION
wire exc_int ;
wire [4: 0] exc_code;
wire [31:0] pc;
//Int
wire has_int ;
wire count_cmp_eq;

//Handle Exception
reg  flush_line;
reg  [31:0] next_pc;


//ERET
wire op_eret = ms_op == `VIE_OP_ERET;

//control signals
reg ws_valid_r;
wire ws_cango;

assign ws_cango = 1'b1;
assign ws_allowin = !ws_valid_r || ws_cango;

always @(posedge clock) begin
    if(reset) begin
        ws_valid_r <= 1'b0;
    end else if(ws_allowin) begin
        if(flush_happen) ws_valid_r <= 1'b0;
        else ws_valid_r <= ms_valid;
    end

    if(ms_valid && ws_allowin) begin
        msbus_r <= msbus_i;
    end
end



//Int
assign count_cmp_eq = c0_compare == c0_count;
assign has_int      = ((c0_cause_ip[7:0] & c0_status_im[7:0]) != 8'h00) && (c0_status_ie==1'b1) && (c0_status_exl==1'b0);

//Exception
assign exc_adel = ms_exc[5];
assign exc_ades = ms_exc[4];
assign exc_ov   = ms_exc[3];
assign exc_sys  = ms_exc[2];
assign exc_bp   = ms_exc[1];
assign exc_ri   = ms_exc[0];
assign exc_int  = has_int;
assign bd       = ms_bd;
assign pc       = ms_pc;
assign exc = ws_valid_r & (exc_adel | exc_ades | exc_ov | exc_sys | exc_bp | exc_ri | exc_int | 0);
assign exc_code = exc_int  ? 5'b00000 :
                  exc_adel ? 5'b00100 :
                  exc_ades ? 5'b00101 :
                  exc_sys  ? 5'b01000 :
                  exc_bp   ? 5'b01001 :
                  exc_ri   ? 5'b01010 :
                  exc_ov   ? 5'b01100 : 5'b00000;

always @(posedge clock) begin
    if(reset) begin
        exc_r <= 1'b0;
        exl_r <= 1'b0;
    end else begin
        exc_r <= exc;
        exl_r <= c0_status_exl;
    end
end

//Cp0 Operation
assign cp0_ren      = ws_op == `VIE_OP_MFC0;
assign cp0_wen      = ws_valid_r && op_mtc0 && !flush_happen;
assign cp0_addr     = ms_cp0_addr;
assign cp0_raddr    = ws_op == `VIE_OP_MFC0 ? ws_cp0_addr : 8'h0;
assign cp0_waddr    = op_mtc0 ? cp0_addr : 8'h0;
assign cp0_wr_value = ms_res[31:0];

assign cp0_rd_value = 
            {32{ws_cp0_addr==`CR_STATUS }}&status_value 
          | {32{ws_cp0_addr==`CR_CAUSE  }}&cause_value
          | {32{ws_cp0_addr==`CR_EPC    }}&epc_value
          | {32{ws_cp0_addr==`CR_COUNT  }}&count_value
          | {32{ws_cp0_addr==`CR_COMPARE}}&compare_value
          ;

always @(posedge clock) begin
    //Count
    if(reset) tick <= 1'b0;
    else      tick <= ~tick;

    if(cp0_wen && cp0_waddr==`CR_COUNT)
        c0_count <= cp0_wr_value;
    else if(tick)
        c0_count <= c0_count + 1'b1;
    
    //Compare
    if(reset)
        c0_compare <= 32'h0;
    else if(cp0_wen && cp0_waddr==`CR_COMPARE)
        c0_compare <= cp0_wr_value;


    //Status
    
    //bev
    if(reset) c0_status_bev <= 1'b1;

    //IM
    if(cp0_wen && cp0_waddr==`CR_STATUS)
        c0_status_im <= cp0_wr_value[15:8];

    //EXL
    if(reset)
        c0_status_exl <= 1'b0;
    else if(exc)
        c0_status_exl <= 1'b1;
    else if(op_eret)
        c0_status_exl <= 1'b0;
    else if(cp0_wen && cp0_waddr==`CR_STATUS)
        c0_status_exl <= cp0_wr_value[1];
    
    //IE
    if(reset)
        c0_status_ie <= 1'b0;
    else if(cp0_wen && cp0_waddr==`CR_STATUS)
        c0_status_exl <= cp0_wr_value[0];

    //Cause
    //BD
    if(reset) 
        c0_cause_bd <= 1'b0;
    else if(exc && !c0_status_exl)
        c0_cause_bd <= bd;
    
    //TI
    if(reset)
        c0_cause_ti <= 1'b0;
    else if(cp0_wen && cp0_waddr==`CR_COMPARE)
        c0_cause_ti <= 1'b0;
    else if(count_cmp_eq)
        c0_cause_ti <= 1'b1;

    //IP7-IP2
    if(reset)
        c0_cause_ip[7:2] <= 6'b0;
    else begin
        c0_cause_ip[7]   <= ext_int_in[5] | c0_cause_ti;
        c0_cause_ip[6:2] <= ext_int_in[4:0];
    end

    //IP1-IP0
    if(reset)
        c0_cause_ip[1:0] <= 2'b0;
    else if(cp0_wen && cp0_waddr==`CR_CAUSE)
        c0_cause_ip[1:0] <= cp0_wr_value;
    
    //EXcode
    if(reset)
        c0_cause_excode <= 5'b0;
    else if(exc)
        c0_cause_excode <= exc_code;

    //EPC
    if(exc && !c0_status_exl)
        c0_epc <= bd ? pc-3'h4 : pc;
    else if(cp0_wen && cp0_waddr==`CR_EPC)
        c0_epc <= cp0_wr_value;

end


//Handle Exception
always @(posedge clock) begin
    if(reset) begin
        flush_line <= 1'b0;
    end else if(!c0_status_exl && exc) begin
        flush_line <= 1'b1;
        next_pc    <= 32'hBFC00380;
    end else if(op_eret) begin
        flush_line <= 1'b1;
        next_pc    <= c0_epc;
    end else begin
        flush_line <= 1'b0;
    end
end


always @(posedge clock) begin
    if(reset)
        res_src <= 1'b0;
    else if(op_mfc0)
        res_src <= 1'b1;
    else 
        res_src <= 1'b0;
end

assign flush_happen = flush_line;
assign flush_target = next_pc;

//wstate
assign state_kernel = exl_r;
assign state_exc    = exc_r;


assign rf_we    = (ws_dest[6:5] == 2'b00) && ws_valid_r && (!flush_happen);
assign rf_waddr = ws_dest[4:0];
// assign rf_wdata = op_mfc0 ? cp0_rd_value : ws_res;
assign rf_wdata = res_src ? cp0_rd_value : ws_res;

assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_wen   = {4{rf_we&&(!flush_happen)}};
assign debug_wb_rf_wnum  = rf_waddr;
assign debug_wb_rf_wdata = rf_wdata;

endmodule