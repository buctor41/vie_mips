`include "vie_define.h"

module vie_exe_stage(
    clock,
    reset,

    issuebus_i,
    ms_allowin,
    rsbus_o,
    es_allowin,
    data_ifc_o,
    rstatus_o
);
input                        clock;
input                        reset;

input  [`Vissuebus       -1:0] issuebus_i;
input                          ms_allowin;
output [`Vrsbus          -1:0] rsbus_o   ;
output                         es_allowin;
output [`Vtoifcbus       -1:0] data_ifc_o;
output [`Vrstatus        -1:0] rstatus_o ;

wire        issue_valid = issuebus_i[146:146];
wire [31:0]    issue_pc = issuebus_i[145:114];
wire [7 :0]    issue_op = issuebus_i[113:106];
wire [6 :0]  issue_dest = issuebus_i[105: 99];
wire        issue_v1_en = issuebus_i[ 98: 98];
wire [31:0]    issue_v1 = issuebus_i[ 97: 66];
wire        issue_v2_en = issuebus_i[ 65: 65];
wire [31:0]    issue_v2 = issuebus_i[ 64: 33];
wire        issue_v3_en = issuebus_i[ 32: 32];
wire [31:0]    issue_v3 = issuebus_i[ 31:  0];

//rsbus unknown
wire        rs_valid;
wire        rs_is_load;
wire [6: 0] rs_dest;
wire [31:0] rs_fixres;
wire [31:0] rs_pc;

assign rsbus_o[72:72] = rs_valid;
assign rsbus_o[71:71] = rs_is_load;
assign rsbus_o[70:64] = rs_dest;
assign rsbus_o[63:32] = rs_fixres;
assign rsbus_o[31: 0] = rs_pc;

wire        data_sram_en   ;
wire [3 :0] data_sram_wen  ;
wire [31:0] data_sram_addr ;
wire [31:0] data_sram_wdata;

assign data_ifc_o[68:68] = data_sram_en   ;
assign data_ifc_o[67:64] = data_sram_wen  ;
assign data_ifc_o[63:32] = data_sram_addr ;
assign data_ifc_o[31: 0] = data_sram_wdata;

//control signals
reg  es_valid_r;
wire es_cango;
wire es_to_ms_valid;


//internal sinals
reg [`Vissuebus - 1:0] issue_r;       

wire        es_valid;
wire [31:0] es_pc;
wire [ 7:0] es_op;
wire [ 6:0] es_dest;
wire        es_v1_en;
wire [31:0] es_v1;
wire        es_v2_en;
wire [31:0] es_v2;
wire        es_v3_en;
wire [31:0] es_v3;

assign {es_valid,
        es_pc,
        es_op,
        es_dest,
        es_v1_en,
        es_v1,
        es_v2_en,
        es_v2,
        es_v3_en,
        es_v3
       } = issue_r;

wire [31:0] fix_res;


reg  [31:0] hi_r,lo_r;
//32-bit ADD 
wire [32:0] adder_a;
wire [32:0] adder_b;
wire        adder_cin;
wire [31:0] adder_res;
wire        adder_cout;
wire        adder_lt;
wire        adder_ltu;
wire        adder_ge;
wire        adder_geu;

//bitwise logic
wire [31:0] logic_ina;
wire [31:0] logic_inb;
wire [31:0] and_out;
wire [31:0] or_out;
wire [31:0] xor_out;
wire [31:0] nor_out;


//Shifter
wire [31:0] shft0_a;
wire [ 4:0] shft0_b;
wire [31:0] shft0_l_res;
wire        shft0_r_sign;
wire        shftr_adding_bit;
wire [31:0] shft0_r_res;

//mul
wire [31:0] mul_a;
wire [31:0] mul_b;
wire [63:0] unsigned_prod;
wire [63:0] signed_prod;
wire [31:0] mul_prod_h;
wire [31:0] mul_prod_l;


//EXECUTION
wire op_add     = es_op == `VIE_OP_ADD;
wire op_addu    = es_op == `VIE_OP_ADDU;
wire op_sub     = es_op == `VIE_OP_SUB;
wire op_subu    = es_op == `VIE_OP_SUBU;

wire op_slt     = es_op == `VIE_OP_SLT;
wire op_sltu    = es_op == `VIE_OP_SLTU;

wire op_mov     = es_op == `VIE_OP_MOV;


wire op_and     = es_op == `VIE_OP_AND;
wire op_or      = es_op == `VIE_OP_OR;
wire op_xor     = es_op == `VIE_OP_XOR;
wire op_nor     = es_op == `VIE_OP_NOR;

wire op_sll     = es_op == `VIE_OP_SLL;
wire op_srl     = es_op == `VIE_OP_SRL;
wire op_sra     = es_op == `VIE_OP_SRA;

wire op_mult    = es_op == `VIE_OP_MULT;
wire op_multu   = es_op == `VIE_OP_MULTU;
wire op_div     = es_op == `VIE_OP_DIV;
wire op_divu    = es_op == `VIE_OP_DIVU;



wire op_mfhi    = es_op == `VIE_OP_MFHI;
wire op_mflo    = es_op == `VIE_OP_MFLO;
wire op_mthi    = es_op == `VIE_OP_MTHI;
wire op_mtlo    = es_op == `VIE_OP_MTLO;



wire op_lw      = es_op == `VIE_OP_LW;
wire op_sw      = es_op == `VIE_OP_SW;

wire op_jal     = es_op == `VIE_OP_JAL;

wire mult_op    = op_mult | op_multu;

wire div_op     = op_div  | op_divu;

wire link_op    = op_jal;

wire sub_op     = op_subu | op_slt | op_sltu | op_sub;

wire load_op    = op_lw;

wire store_op   = op_sw;

wire mem_op     = store_op | load_op;

wire add_op     = op_addu | mem_op | op_add;

wire logic_op   = op_and | op_or | op_xor | op_nor;

wire shft_op   = op_sll | op_srl | op_sra;

wire addsub_op  = op_addu | op_subu | op_add | op_sub;




//Fix 32-bit ADD
assign adder_a = (add_op | sub_op) ? {1'b0,es_v1} :
                           link_op ? {1'b0,es_pc} :
                                     33'h0;

assign adder_b = add_op ? {1'b0, es_v2}  :
                 sub_op ? {1'b0, ~es_v2} :
                link_op ? 33'h8          :
                          33'h0          ;
assign adder_cin = sub_op;
assign {adder_cout,adder_res} = adder_a + adder_b + adder_cin;

assign adder_lt  = es_v1[31]&adder_res[31] | 
                   es_v1[31]&(~es_v2[31])  |
                   adder_res[31]&(~es_v2[31]);

assign adder_ltu = ~adder_cout;

assign adder_ge  = ~adder_lt;

assign adder_geu = adder_cout;


//还未考虑溢出

//Bitwise logic
assign logic_ina = logic_op ? es_v1 : 32'h0;
assign logic_inb = logic_op ? es_v2 : 32'h0;

assign and_out   = logic_ina & logic_inb;

assign or_out    = logic_ina | logic_inb;

assign xor_out   = logic_ina ^ logic_inb;

assign nor_out   = ~or_out;


//fix 32x32->64 MUL
assign mul_a = es_v1;
assign mul_b = es_v2;

assign unsigned_prod = mul_a * mul_b;
assign signed_prod   = $signed(mul_a) * $signed(mul_b);
assign {mul_prod_h,mul_prod_l} = mult_op ? op_mult ? signed_prod   : 
                                          op_multu ? unsigned_prod :
                                                     64'h0         :
                                                     64'h0;


//div
wire        div_not_complete;
wire [31:0] div_a;
wire [31:0] div_b;
wire [31:0] div_quot;
wire [31:0] div_rem;



assign div_a = div_op ? es_v1 : 32'h0;
assign div_b = div_op ? es_v2 : 32'h0;
//Signed div
reg  [1:0]  signed_div_status  ;
wire        signed_div_ready   ;


wire [31:0] signed_quot;
wire [31:0] signed_rem;



wire        signed_divisor_tvalid;
wire        signed_divisor_tready; 
wire [31:0] signed_divisor_tdata ;

wire        signed_dividend_tvalid;
wire        signed_dividend_tready;
wire [31:0] signed_dividend_tdata ;

wire        signed_dout_valid;
wire [63:0] signed_dout_tdata;



assign signed_dividend_tdata = op_div ? div_a : 32'h0;
assign signed_divisor_tdata  = op_div ? div_b : 32'h0;

assign signed_div_ready = (signed_div_status == 2'b00) && op_div && es_valid_r;

assign signed_divisor_tvalid  = op_div ? signed_div_ready : 1'b0;
assign signed_dividend_tvalid = op_div ? signed_div_ready : 1'b0;

assign {signed_quot,signed_rem} = {64{signed_div_status[1]}} &  signed_dout_tdata;




always @(posedge clock) begin
    if(reset) begin
        signed_div_status <= 2'b00;
    end
    if(signed_divisor_tready == 1'b1 && signed_dividend_tready == 1'b1 && signed_div_ready) begin
        signed_div_status <= 2'b01;
    end else if(signed_dout_valid == 1'b1) begin
        signed_div_status <= 2'b10;
    end else if(signed_div_status == 2'b10) begin
        signed_div_status <= 2'b00;
    end

end

div_signed u0_div_signed(
    .aclk(clock),   .s_axis_divisor_tvalid(signed_divisor_tvalid)  , .s_axis_divisor_tready(signed_divisor_tready),
                    .s_axis_divisor_tdata(signed_divisor_tdata)    , .s_axis_dividend_tvalid(signed_dividend_tvalid), 
                    .s_axis_dividend_tready(signed_dividend_tready), .s_axis_dividend_tdata(signed_dividend_tdata),
                    .m_axis_dout_tvalid(signed_dout_valid)         , .m_axis_dout_tdata(signed_dout_tdata)
);
//unsigned_div


reg  [1:0]  unsigned_div_status;
wire        unsigned_div_ready ;

wire [31:0] unsigned_quot;
wire [31:0] unsigned_rem ;

wire        unsigned_divisor_tvalid;
wire        unsigned_divisor_tready;
wire [31:0] unsigned_divisor_tdata;

wire        unsigned_dividend_tvalid;
wire        unsigned_dividend_tready;
wire [31:0] unsigned_dividend_tdata;

wire        unsigned_dout_tvalid;
wire [63:0] unsigned_dout_tdata;


assign unsigned_dividend_tdata = op_divu ? div_a : 32'h0;
assign unsigned_divisor_tdata = op_divu ? div_b : 32'h0;

assign unsigned_div_ready = (unsigned_div_status == 2'b00) && op_divu && es_valid_r;

assign unsigned_dividend_tvalid = op_divu ? unsigned_div_ready : 1'b0;
assign unsigned_divisor_tvalid  = op_divu ? unsigned_div_ready : 1'b0;

assign {unsigned_quot,unsigned_rem} = {64{unsigned_div_status[1]}} & unsigned_dout_tdata;

always @(posedge clock) begin
    if(reset) begin
        unsigned_div_status <= 2'b00;
    end
    if(unsigned_divisor_tready == 1'b1 && unsigned_dividend_tready && unsigned_div_ready) begin
        unsigned_div_status <= 2'b01;
    end else if(unsigned_dout_tvalid == 1'b1) begin
        unsigned_div_status <= 2'b10;
    end else if(unsigned_div_status == 2'b10) begin
        unsigned_div_status <= 2'b00;
    end
end
div_unsigned u1_div_unsigned(
    .aclk(clock),   .s_axis_divisor_tvalid(unsigned_divisor_tvalid)  , .s_axis_divisor_tready(unsigned_divisor_tready),
                    .s_axis_divisor_tdata(unsigned_divisor_tdata)    , .s_axis_dividend_tvalid(unsigned_dividend_tvalid), 
                    .s_axis_dividend_tready(unsigned_dividend_tready), .s_axis_dividend_tdata(unsigned_dividend_tdata),
                    .m_axis_dout_tvalid(unsigned_dout_tvalid)         , .m_axis_dout_tdata(unsigned_dout_tdata)
);

assign {div_quot,div_rem} = op_div ? {signed_quot,signed_rem}    :
                           op_divu ? {unsigned_quot,unsigned_rem}:
                                     64'h0;

assign div_not_complete = signed_div_ready   || (signed_div_status == 2'b01) || 
                          unsigned_div_ready || (unsigned_div_status == 2'b01);

// assign div_a = es_v1;
// assign div_b = es_v2;

// assign unsigned_quot = op_divu ? div_a / div_b :
//                                  32'h0;
// assign unsigned_rem  = op_divu ? div_a % div_b :
//                                  32'h0;

// assign signed_quot   = op_div ? $signed(div_a) / $signed(div_b) :
//                                 32'h0;
// assign signed_rem    = op_div ? $signed(div_a) % $signed(div_b) :
//                                 32'h0;

// assign div_quot      = div_op ? op_div  ?  signed_quot :
//                                 op_divu ? unsigned_quot:
//                                             32'h0      :
//                                             32'h0;
// assign div_rem       = div_op ? op_div  ?  signed_rem  :
//                                 op_divu ?  unsigned_rem:
//                                            32'h0       :
//                                            32'h0;


//Shifter
assign shft0_a =  shft_op ? es_v1 : 32'h0;

assign shft0_b =  shft_op ? es_v2[4:0] :
                            5'h0;

assign shft0_l_res = shft0_a << shft0_b;

assign shft0_r_sign= op_sra;

assign shftr_adding_bit = shft0_r_sign & shft0_a[31];

wire [31:0] shftr_tmp1 = {32{shft0_b[1:0]==2'b00}}&{                      shft0_a[31:0]} |
                         {32{shft0_b[1:0]==2'b01}}&{{1{shftr_adding_bit}},shft0_a[31:1]} |
                         {32{shft0_b[1:0]==2'b10}}&{{2{shftr_adding_bit}},shft0_a[31:2]} |
                         {32{shft0_b[1:0]==2'b11}}&{{3{shftr_adding_bit}},shft0_a[31:3]} ;
 
wire [31:0] shftr_tmp2 = {32{shft0_b[3:2]==2'b00}}&{                       shftr_tmp1[31: 0]} |
                         {32{shft0_b[3:2]==2'b01}}&{{ 4{shftr_adding_bit}},shftr_tmp1[31: 4]} |
                         {32{shft0_b[3:2]==2'b10}}&{{ 8{shftr_adding_bit}},shftr_tmp1[31: 8]} |
                         {32{shft0_b[3:2]==2'b11}}&{{12{shftr_adding_bit}},shftr_tmp1[31:12]} ;
 
wire [31:0] shftr_tmp3 = shft0_b[4] ? {{16{shftr_adding_bit}},shftr_tmp2[31:16]} : shftr_tmp2;

assign shft0_r_res = shftr_tmp3;



assign fix_res = {32{addsub_op}} & adder_res 
               | {32{op_slt   }} & {31'h0,adder_lt}
               | {32{op_sltu  }} & {31'h0,adder_ltu}
               | {32{op_mov   }} & es_v1
               | {32{op_and   }} & and_out
               | {32{op_or    }} & or_out
               | {32{op_xor   }} & xor_out
               | {32{op_nor   }} & nor_out
               | {32{op_sll   }} & shft0_l_res
               | {32{op_srl   }} & shft0_r_res 
               | {32{op_sra   }} & shft0_r_res
               | {32{link_op  }} & adder_res
               | {32{op_mfhi  }} & hi_r
               | {32{op_mflo  }} & lo_r;



//L / S
assign data_sram_en    = es_valid_r && (store_op || load_op);
assign data_sram_wen   = (es_valid_r && store_op) ? 4'hf : 4'h0;
assign data_sram_addr  = adder_res;
assign data_sram_wdata = es_v3;





//control logic
assign es_cango       = !div_not_complete;
assign es_allowin     = !es_valid_r || es_cango && ms_allowin;
assign es_to_ms_valid = es_valid_r && es_cango;


always @(posedge clock) begin
    if(reset) begin
        es_valid_r <= 1'b0;
    end else if(es_allowin) begin
        es_valid_r <= issue_valid;
    end

    if(issue_valid && es_allowin) begin
        issue_r <= issuebus_i;
    end
end

//Reg hi/lo
always @(posedge clock) begin
    if(es_valid_r && !div_not_complete) begin
        if(div_op) begin
            {hi_r,lo_r} <= {div_rem, div_quot};
        end if(mult_op) begin
            {hi_r,lo_r} <= {mul_prod_h,mul_prod_l};
        end else if(op_mthi) begin
            hi_r <= es_v1;
        end else if(op_mtlo) begin
            lo_r <= es_v1;
        end
    end
end



//OUTPUT
assign rs_valid   = es_to_ms_valid;
assign rs_is_load = load_op;
assign rs_dest    = es_dest;
assign rs_fixres  = fix_res;
assign rs_pc      = es_pc;

//status
assign rstatus_o[40:40] = es_valid_r;
assign rstatus_o[39:39] = load_op;
assign rstatus_o[38:32] = es_dest;
assign rstatus_o[31: 0] = fix_res;
endmodule