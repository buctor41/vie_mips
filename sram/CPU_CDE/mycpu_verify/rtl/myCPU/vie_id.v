`include "vtools.h"
module vie_id_stage(
    clock     ,
    reset     ,

    es_allowin,
    fsbus_i   ,
    wsbus_i   ,
    ds_allowin,
    rstatus_i ,
    mstatus_i ,
    brbus_o   ,
    issuebus_o,
    flushbus_i,
    estate_i   
);
/*
 * PORT declartion
 */
input                   clock;
input                   reset;

input                      es_allowin;
input  [`Vfsbus      -1:0] fsbus_i   ;
input  [`Vwsbus      -1:0] wsbus_i   ;
input  [`Vrstatus    -1:0] rstatus_i ;
input  [`Vmstatus    -1:0] mstatus_i ;
input  [`Vflushbus   -1:0] flushbus_i;
output [`Vbrbus      -1:0] brbus_o   ;
output                     ds_allowin;
output [`Vissuebus   -1:0] issuebus_o;
input  [`Vestate     -1:0] estate_i  ;

wire fs_to_ds_valid = fsbus_i[102:102];
wire [31:0] fs_baddr= fsbus_i[101: 70];
wire [5 :0] fs_exc  = fsbus_i[69 : 64];
wire [31:0] fs_pc   = fsbus_i[63 : 32];
wire [31:0] fs_inst = fsbus_i[31 :  0];

wire        rf_we    = wsbus_i[37:37];
wire [ 4:0] rf_waddr = wsbus_i[36:32]; 
wire [31:0] rf_wdata = wsbus_i[31: 0] ;

wire        es_valid = rstatus_i[41:41];
wire        es_mfc0  = rstatus_i[40:40];
wire        es_load  = rstatus_i[39:39];
wire        es_wen   = !rstatus_i[38:38];
wire [4:0]  es_waddr = rstatus_i[36:32];
wire [31:0] es_wdata = rstatus_i[31: 0];

wire        ms_valid = mstatus_i[40:40];
wire        ms_mfc0  = mstatus_i[39:39];
wire        ms_wen   = !mstatus_i[38:38];
wire [4:0]  ms_waddr = mstatus_i[36:32];
wire [31:0] ms_wdata = mstatus_i[31: 0];


wire        flush_taken  = flushbus_i[32:32];
wire [31:0] flush_target = flushbus_i[31: 0];



wire        br_taken ;
wire [31:0] br_target;
assign brbus_o[32:32] = br_taken;
assign brbus_o[31: 0] = br_target;


wire        state_kernel = estate_i[1:1];
wire        state_exc    = estate_i[0:0];

wire        issue_valid;
wire [31:0] issue_baddr;
wire        issue_bd   ;
wire [ 5:0] issue_exc  ;
wire [31:0] issue_pc   ;
wire [7 :0] issue_op   ;
wire [6 :0] issue_dest ;
wire        issue_v1_en;
wire [31:0] issue_v1   ;
wire        issue_v2_en;
wire [31:0] issue_v2   ;
wire        issue_v3_en;
wire [31:0] issue_v3   ;

assign issuebus_o[185:185] = issue_valid;
assign issuebus_o[184:153] = issue_baddr;
assign issuebus_o[152:152] = issue_bd   ;
assign issuebus_o[151:146] = issue_exc  ;
assign issuebus_o[145:114] = issue_pc   ;
assign issuebus_o[113:106] = issue_op   ;
assign issuebus_o[105: 99] = issue_dest ;
assign issuebus_o[ 98: 98] = issue_v1_en;
assign issuebus_o[ 97: 66] = issue_v1   ;
assign issuebus_o[ 65: 65] = issue_v2_en;
assign issuebus_o[ 64: 33] = issue_v2   ;
assign issuebus_o[ 32: 32] = issue_v3_en;
assign issuebus_o[ 31:  0] = issue_v3   ;





//control signals
reg  ds_valid_r;
wire ds_cango;
wire ds_to_es_valid;
reg  [31:0] ds_pc_r;
reg  [31:0] ds_inst_r;
reg  [ 5:0] ds_exc_r;
reg  [31:0] ds_baddr_r;
//internal logic and signals
wire        vid_valid         ;
wire [31:0] vid_pc            ;
wire [ 7:0] vid_op            ;
wire [ 6:0] vid_dest          ;
wire        vid_bd            ;

wire [ 5:0] vid_exc           ;
wire        vid_ex_sys        ;
wire        vid_ex_bp         ;
wire        vid_ex_ri         ;
wire [31:0] vid_baddr         ;

assign vid_valid = ds_to_es_valid;
assign vid_pc    = ds_pc_r;

wire        vid_v1_rs         ;
wire        vid_v1_rt         ;
wire        vid_v1_imm_upper  ;

wire        vid_v2_rs         ;
wire        vid_v2_rt         ;
wire        vid_v2_imm_sign_ex;
wire        vid_v2_imm_zero_ex;
wire        vid_v2_imm_sa     ;
wire        vid_v2_sel        ;

wire        vid_v3_rs         ;
wire        vid_v3_rt         ;
wire        vid_v3_imm_offset ;
wire        vid_v3_imm_instr_index;


wire        vid_dest_rd       ;
wire        vid_dest_rt       ;
wire        vid_dest_r31     ;

wire v1_en;
wire v2_en;
wire v3_en;

wire [31:0] v1;
wire [31:0] v2;
wire [31:0] v3;

wire [ 5:0] op              = ds_inst_r[31:26];
wire [ 4:0] rs              = ds_inst_r[25:21];
wire [ 4:0] rt              = ds_inst_r[20:16];
wire [ 4:0] rd              = ds_inst_r[15:11];
wire [ 4:0] sa              = ds_inst_r[10: 6];
wire [ 5:0] func            = ds_inst_r[ 5: 0];
wire [15:0] imm             = ds_inst_r[15: 0];
wire [15:0] offset          = ds_inst_r[15: 0];
wire [ 2:0] cp0_sel         = ds_inst_r[ 2: 0];
wire [25:0] instr_index     = ds_inst_r[25: 0];

wire [63:0] op_d  ;
wire [31:0] rs_d  ;
wire [31:0] rt_d  ;
wire [31:0] rd_d  ;
wire [31:0] sa_d  ;
wire [63:0] func_d;
wire rs_eq_rt;
wire rs_eq_z;
wire rs_lt_z;
wire rs_ge_z;

v_decoder_6_64 u0_d6to64(.in(op[5:0]), .out(op_d[63:0] ));
v_decoder_5_32 u0_d5to32(.in(rs[4:0]), .out(rs_d[31:0] ));
v_decoder_5_32 u1_d5to32(.in(rt[4:0]), .out(rt_d[31:0] ));
v_decoder_5_32 u2_d5to32(.in(rd[4:0]), .out(rd_d[31:0] ));
v_decoder_5_32 u3_d5to32(.in(sa[4:0]), .out(sa_d[31:0] ));
v_decoder_6_64 u1_d6to64(.in(func[5:0]), .out(func_d[63:0] ));


//bd
reg bd;

//exc
wire exc;
wire inc;

wire inst_cpu;
//cpu arithmetic operation
wire inst_add               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h20];

wire inst_addi              = op_d[6'h08];

wire inst_addu              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h21];

wire inst_addiu             = op_d[6'h09];

wire inst_sub               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h22];

wire inst_subu              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h23];

wire inst_slt               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h2a];

wire inst_slti              = op_d[6'h0a];

wire inst_sltu              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h2b];

wire inst_sltiu             = op_d[6'h0b];

wire inst_mult              = op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h18];

wire inst_multu             = op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h19];

wire inst_div               = op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h1a];

wire inst_divu              = op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h1b];

wire inst_ae                = inst_add   | inst_addi | inst_addu | inst_addiu | inst_sub  |
                              inst_subu  | inst_slt  | inst_slti | inst_sltiu | inst_mult |
                              inst_multu | inst_div  | inst_divu | inst_sltu  ;
//cpu logic operation
wire inst_lui               = op_d[6'h0f]&rs_d[5'h00];

wire inst_and               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h24];

wire inst_andi              = op_d[6'h0c];

wire inst_or                = op_d[6'h00]&sa_d[5'h00]&func_d[6'h25];

wire inst_ori               = op_d[6'h0d];

wire inst_xor               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h26];

wire inst_xori              = op_d[6'h0e];

wire inst_nor               = op_d[6'h00]&sa_d[5'h00]&func_d[6'h27];

wire inst_logic             = inst_lui | inst_and  | inst_andi | inst_or | inst_ori |
                              inst_xor | inst_xori | inst_nor  ;


//cpu branch and jump
wire inst_beq               = op_d[6'h04];

wire inst_bgez              = op_d[6'h01]&rt_d[5'h01];

wire inst_bgezal            = op_d[6'h01]&rt_d[5'h11];

wire inst_bgtz              = op_d[6'h07]&rt_d[5'h00];

wire inst_blez              = op_d[6'h06]&rt_d[5'h00];

wire inst_bltz              = op_d[6'h01]&rt_d[5'h00];

wire inst_bltzal            = op_d[6'h01]&rt_d[5'h10];

wire inst_bne               = op_d[6'h05];


wire inst_j                 = op_d[6'h02];

wire inst_jal               = op_d[6'h03];

wire inst_jr                = op_d[6'h00]&rt_d[5'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h08];

wire inst_jalr              = op_d[6'h00]&rt_d[5'h00]&sa_d[5'h00]&func_d[6'h09];


wire inst_br                = inst_beq  | inst_bne  | inst_bgezal | inst_bltzal 
                            | inst_bgez | inst_bgtz | inst_blez   | inst_bltz
                            | inst_j    | inst_jal  | inst_jalr   | inst_jr 
                            ;

//cpu L/S 
wire inst_lb                = op_d[6'h20];

wire inst_lh                = op_d[6'h21];

wire inst_lwl               = op_d[6'h22];

wire inst_lw                = op_d[6'h23];

wire inst_lbu               = op_d[6'h24];

wire inst_lhu               = op_d[6'h25];

wire inst_lwr               = op_d[6'h26];

wire inst_sb                = op_d[6'h28];

wire inst_sh                = op_d[6'h29];

wire inst_swl               = op_d[6'h2a];

wire inst_sw                = op_d[6'h2b];

wire inst_swr               = op_d[6'h2e];

wire inst_io                = inst_lb  | inst_lh | inst_lwl | inst_lbu | inst_lhu |
                              inst_lwr | inst_sb | inst_sh  | inst_swl | inst_sw  |
                              inst_swr | inst_lw ;

//cpu move
wire inst_mfhi              = op_d[6'h00]&rs_d[5'h00]&rt_d[5'h00]&sa_d[5'h00]&func_d[6'h10];

wire inst_mflo              = op_d[6'h00]&rs_d[5'h00]&rt_d[5'h00]&sa_d[5'h00]&func_d[6'h12];

wire inst_mthi              = op_d[6'h00]&rt_d[5'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h11];

wire inst_mtlo              = op_d[6'h00]&rt_d[5'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h13];

wire inst_hl                 = inst_mfhi | inst_mflo | inst_mthi | inst_mtlo;

//cpu shift
wire inst_sll               = op_d[6'h00]&rs_d[5'h00]&func_d[6'h00];

wire inst_sllv              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h04];

wire inst_srl               = op_d[6'h00]&rs_d[5'h00]&func_d[6'h02];

wire inst_srlv              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h06];

wire inst_sra               = op_d[6'h00]&rs_d[5'h00]&func_d[6'h03];

wire inst_srav              = op_d[6'h00]&sa_d[5'h00]&func_d[6'h07];

wire inst_shift             = inst_sll | inst_sllv | inst_srl | inst_srlv | inst_sra | inst_srav;

//cpu trap
wire inst_break             = op_d[6'h00]&func_d[6'h0d];

wire inst_syscall           = op_d[6'h00]&func_d[6'h0c];

wire inst_trap              = inst_break | inst_syscall;

//Privileged Instructions
wire inst_eret              = op_d[6'h10]&rs_d[5'h10]&rt_d[5'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h18];

wire inst_mfc0              = op_d[6'h10]&rs_d[5'h00]&sa_d[5'h00]&(func[5:3]==3'b0);

wire inst_mtc0              = op_d[6'h10]&rs_d[5'h04]&sa_d[5'h0]&&(func[5:3]==3'b0);

wire inst_pri               = inst_eret | inst_mfc0 | inst_mtc0;

//branch inst


//internal aluop_code  generation
assign vid_op[7] = inst_lw   | inst_sw   | inst_lb | inst_lbu | inst_lh | inst_lhu |
                   inst_lwl  | inst_lwr  | inst_sb | inst_sh  | inst_swl| inst_swr |
                   inst_mtc0 | inst_mfc0 | inst_eret;


assign vid_op[6] = 1'b0;


assign vid_op[5] = inst_sltu | inst_slt  | inst_beq  | inst_bne   | inst_jal   |
                   inst_jr   | inst_slti | inst_sltiu| inst_bgez  | inst_bgtz  |
                   inst_blez | inst_bltz | inst_j    | inst_bltzal| inst_bgezal|
                   inst_jalr | inst_lwl  | inst_lwr  | inst_swl   | inst_swr;

assign vid_op[4] = inst_or   | inst_and   | inst_subu  | inst_addiu | inst_addu |
                   inst_xor  | inst_nor   | inst_sll   | inst_srl   | inst_sra  |
                   inst_lw   | inst_sw    | inst_beq   | inst_bne   | inst_add  |
                   inst_addi | inst_sub   | inst_andi  | inst_ori   | inst_xori |
                   inst_sllv | inst_srlv  | inst_srav  | inst_mult  | inst_multu|
                   inst_div  | inst_divu  | inst_bgez  | inst_bgtz  | inst_blez |
                   inst_bltz | inst_bltzal| inst_bgezal| inst_lb    | inst_lbu  | 
                   inst_lh   | inst_lhu   | inst_lwl   | inst_lwr   | inst_sb   | 
                   inst_sh   | inst_swl   | inst_swr;


assign vid_op[3] = inst_or   | inst_and  | inst_subu | inst_addiu | inst_addu |
                   inst_xor  | inst_nor  | inst_sw   | inst_jal   | inst_jr   |
                   inst_add  | inst_addi | inst_sub  | inst_andi  | inst_ori  |
                   inst_xori | inst_mfhi | inst_mflo | inst_mthi  | inst_mtlo |
                   inst_j    | inst_jalr | inst_lwl  | inst_lwr   | inst_sb   |
                   inst_sh   | inst_swl  | inst_swr  | inst_mtc0  | inst_mfc0; 


assign vid_op[2] = inst_or    | inst_and   | inst_sltu | inst_slt | inst_lui |
                   inst_xor   | inst_nor   | inst_jal  | inst_jr  | inst_slti|
                   inst_sltiu | inst_andi  | inst_ori  | inst_xori| inst_mult|
                   inst_multu | inst_div   | inst_divu | inst_mfhi| inst_mflo|
                   inst_mthi  | inst_mtlo  | inst_bgez | inst_bltz| inst_j   | 
                   inst_bltzal| inst_bgezal| inst_jalr | inst_lbu | inst_lhu |
                   inst_mtc0  | inst_mfc0  | inst_eret; 

assign vid_op[1] = inst_sltu | inst_slt | inst_subu  | inst_xor   | inst_nor |
                   inst_srl  | inst_sra | inst_lw    | inst_sw    | inst_jal |
                   inst_sub  | inst_slti| inst_sltiu | inst_xori  | inst_srlv|
                   inst_srav | inst_div | inst_divu  | inst_mthi  | inst_mtlo|
                   inst_bgtz | inst_blez| inst_bltzal| inst_bgezal| inst_jalr|
                   inst_swl  | inst_swr | inst_eret;


assign vid_op[0] = inst_sltu | inst_subu  | inst_addiu | inst_addu | inst_lui |
                   inst_or   | inst_nor   | inst_sll   | inst_sra  | inst_bne |
                   inst_jr   | inst_sltiu | inst_ori   | inst_sllv | inst_srav|
                   inst_multu| inst_divu  | inst_mflo  | inst_mtlo | inst_bgez|
                   inst_bgtz | inst_bgezal| inst_jalr  | inst_lh   | inst_lhu |
                   inst_lwr  | inst_sh    | inst_swr   | inst_mtc0 | inst_eret;


assign vid_v1_rs = inst_addu | inst_addiu | inst_subu  | inst_slt | inst_sltu |
                   inst_and  | inst_or    | inst_xor   | inst_nor | inst_lw   |
                   inst_sw   | inst_beq   | inst_bne   | inst_add | inst_addi |
                   inst_sub  | inst_slti  | inst_sltiu | inst_andi| inst_ori  |
                   inst_xori | inst_mult  | inst_multu | inst_div | inst_divu |
                   inst_mthi | inst_mtlo  | inst_bgez  | inst_bgtz| inst_blez |
                   inst_bltz | inst_bltzal| inst_bgezal| inst_lb  | inst_lbu  |
                   inst_lh   | inst_lhu   | inst_lwl   | inst_lwr | inst_sb   |
                   inst_sh   | inst_swl   | inst_swr;
//cpu arithmetic

assign vid_v1_rt = inst_sll  | inst_srl | inst_sra | inst_sllv | inst_srlv | inst_srav |
                   inst_mtc0;

assign vid_v1_imm_upper = inst_lui;

assign vid_v2_rs = inst_sllv | inst_srlv | inst_srav;


assign vid_v2_rt = inst_addu | inst_subu | inst_slt | inst_sltu | inst_and |
                   inst_or   | inst_xor  | inst_nor | inst_beq  | inst_bne |
                   inst_add  | inst_sub  | inst_mult| inst_multu| inst_div |
                   inst_divu;


assign vid_v2_imm_sign_ex = inst_addiu | inst_lw | inst_sw  | inst_addi | inst_slti |
                            inst_sltiu | inst_lb | inst_lbu | inst_lh   | inst_lhu  |
                            inst_lwl   | inst_lwr| inst_sb  | inst_sh   | inst_swl  |
                            inst_swr;


assign vid_v2_imm_zero_ex = inst_andi | inst_ori | inst_xori;


assign vid_v2_imm_sa      = inst_sll | inst_srl | inst_sra;

assign vid_v2_sel         = inst_mtc0 | inst_mfc0;

assign vid_v3_rs          = inst_jr | inst_jalr;


assign vid_v3_rt          = inst_sw | inst_lwl | inst_lwr | inst_sb | inst_sh | inst_swl | inst_swr;


assign vid_v3_imm_offset  = inst_beq | inst_bne | inst_bgez | inst_bgtz | inst_blez | inst_bltz |
                            inst_bltzal | inst_bgezal;

assign vid_v3_imm_instr_index = inst_jal | inst_j;

//RegFile read
wire        rf_rvalid0;
wire [ 4:0] rf_raddr0 ;
wire [31:0] rf_rvalue0;
wire        rf_rvalid1;
wire [ 4:0] rf_raddr1 ;
wire [31:0] rf_rvalue1;
wire        rf_wvalid0;
wire [ 4:0] rf_waddr0 ;
wire [31:0] rf_wvalue0;
wire        rf_fwd0;        //wb?????????????????????????????????
wire        rf_fwd1;        //wb?????????????????????????????????


wire        ld_fwd0;
wire        ld_fwd1;

wire        es_fwd0;
wire        es_fwd1;

wire        ms_fwd0;
wire        ms_fwd1;

wire        v_load_hazard;

wire        es_mfc0_fwd0;
wire        es_mfc0_fwd1;

wire        ms_mfc0_fwd0;
wire        ms_mfc0_fwd1;

wire        v_mfc0_hazard;

wire        v_hazard;


assign rf_rvalid0 = vid_v1_rs | vid_v1_rt;
assign rf_raddr0  = vid_v1_rt ? rt : rs;

assign rf_rvalid1 = vid_v2_rs | vid_v2_rt | vid_v3_rs | vid_v3_rt;
assign rf_raddr1  = (vid_v2_rs | vid_v3_rs) ? rs : rt;

assign rf_wvalid0 = rf_we;
assign rf_waddr0  = rf_waddr;
assign rf_wvalue0 = rf_wdata;

wire wb_rf_nonzero = rf_waddr0 != 5'h0;

assign ld_fwd0        = es_valid && es_load && es_wen && (es_waddr == rf_raddr0);
assign ld_fwd1        = es_valid && es_load && es_wen && (es_waddr == rf_raddr1);

assign es_fwd0        = es_valid && es_wen && (es_waddr == rf_raddr0);
assign es_fwd1        = es_valid && es_wen && (es_waddr == rf_raddr1);

assign ms_fwd0        = ms_valid && ms_wen && (ms_waddr == rf_raddr0);
assign ms_fwd1        = ms_valid && ms_wen && (ms_waddr == rf_raddr1);

assign rf_fwd0        = rf_wvalid0 && (rf_waddr0 == rf_raddr0) && wb_rf_nonzero;
assign rf_fwd1        = rf_wvalid0 && (rf_waddr0 == rf_raddr1) && wb_rf_nonzero;


assign es_mfc0_fwd0   = es_valid && es_mfc0 && es_wen && (es_waddr == rf_raddr0);
assign es_mfc0_fwd1   = es_valid && es_mfc0 && es_wen && (es_waddr == rf_raddr1);

assign ms_mfc0_fwd0   = ms_valid && ms_mfc0 && ms_wen && (ms_waddr == rf_raddr0);
assign ms_mfc0_fwd1   = ms_valid && ms_mfc0 && ms_wen && (ms_waddr == rf_raddr1);



assign v_load_hazard  = (ld_fwd0 && (vid_v1_rs | vid_v1_rt)) |
                        (ld_fwd1 && (vid_v2_rs | vid_v2_rt)) |
                        (ld_fwd1 && (vid_v3_rs | vid_v3_rt));

assign v_mfc0_hazard  = (es_mfc0_fwd0 && (vid_v1_rs | vid_v1_rt)) |
                        (es_mfc0_fwd1 && (vid_v2_rs | vid_v2_rt)) |
                        (es_mfc0_fwd1 && (vid_v3_rs | vid_v3_rt)) |
                        (ms_mfc0_fwd0 && (vid_v1_rs | vid_v1_rt)) |
                        (ms_mfc0_fwd1 && (vid_v2_rs | vid_v2_rt)) |
                        (ms_mfc0_fwd1 && (vid_v3_rs | vid_v3_rt)) ;

assign v_hazard = v_load_hazard | v_mfc0_hazard;

vie_heap_2r1w_32x32 u0_rf_heap(.clock(clock),
                               .ren0(rf_rvalid0), .raddr0(rf_raddr0), .rvalue0(rf_rvalue0),
                               .ren1(rf_rvalid1), .raddr1(rf_raddr1), .rvalue1(rf_rvalue1),
                               .wen0(rf_wvalid0), .waddr0(rf_waddr0), .wvalue0(rf_wvalue0)
                              );

assign v1_en = (vid_v1_rs | vid_v1_rt | vid_v1_imm_upper);

// assign v1    = (rf_rvalid0 & rf_fwd0) ? rf_wvalue0 :
//                          (rf_rvalid0) ? rf_rvalue0 :
//                    (vid_v1_imm_upper) ? :
//                                         32'h0;          //??????????????????
assign v1    = ((vid_v1_rs | vid_v1_rt) & ld_fwd0) ? 32'h0            :  //load??????????????????
               ((vid_v1_rs | vid_v1_rt) & es_fwd0) ? es_wdata         :  
               ((vid_v1_rs | vid_v1_rt) & ms_fwd0) ? ms_wdata         :
               ((vid_v1_rs | vid_v1_rt) & rf_fwd0) ? rf_wvalue0       :
                           (vid_v1_rs | vid_v1_rt) ? rf_rvalue0       :
                                (vid_v1_imm_upper) ? {imm[15:0],16'h0}:
                                                     32'h0;


assign v2_en = (vid_v2_rs     | vid_v2_rt | vid_v2_imm_zero_ex | vid_v2_imm_sign_ex| 
                vid_v2_imm_sa | vid_v2_sel);

assign v2    = ((vid_v2_rs | vid_v2_rt) & ld_fwd1) ? 32'h0                        :
               ((vid_v2_rs | vid_v2_rt) & es_fwd1) ? es_wdata                     :
               ((vid_v2_rs | vid_v2_rt) & ms_fwd1) ? ms_wdata                     :
               ((vid_v2_rs | vid_v2_rt) & rf_fwd1) ? rf_wvalue0                   :
                           (vid_v2_rs | vid_v2_rt) ? rf_rvalue1                   :
                              (vid_v2_imm_sign_ex) ? {{16{imm[15]}},imm[15:0]}    :
                              (vid_v2_imm_zero_ex) ? {16'h0,imm[15:0]}            :
                                   (vid_v2_imm_sa) ? {27'h0,sa[4:0]}              :
                                      (vid_v2_sel) ? {24'h0,rd[4:0],cp0_sel[2:0]} :
                                                     32'h0;
// assign v2    = ((vid_v2_rs | vid_v2_rt) & rf_fwd1) ? rf_wvalue0 :
//                            (vid_v2_rs | vid_v2_rt) ? rf_rvalue1 :
//                               (vid_v2_imm_sign_ex) ? {{16{imm[15]}},imm[15:0]} :
//                               (vid_v2_imm_zero_ex) ? {16'h0,imm[15:0]} :
//                                    (vid_v2_imm_sa) ? {27'h0,sa[4:0]} :
//                                                      32'h0;             //??????

assign v3_en = (vid_v3_rs | vid_v3_rt | vid_v3_imm_offset | vid_v3_imm_instr_index);

assign v3    = ((vid_v3_rs | vid_v3_rt) & ld_fwd1) ? 32'h0                                 :
               ((vid_v3_rs | vid_v3_rt) & es_fwd1) ? es_wdata                              :
               ((vid_v3_rs | vid_v3_rt) & ms_fwd1) ? ms_wdata                              :
               ((vid_v3_rs | vid_v3_rt) & rf_fwd1) ? rf_wvalue0                            :
                           (vid_v3_rs | vid_v3_rt) ? rf_rvalue1                            :
                               (vid_v3_imm_offset) ? {{14{offset[15]}},offset[15:0],2'b00} :
                                                     {6'h0,instr_index[25:0]};
// assign v3    = ((vid_v3_rs | vid_v3_rt) & rf_fwd1) ? rf_wvalue0 :
//                            (vid_v3_rs | vid_v3_rt) ? rf_rvalue1 :
//                                (vid_v3_imm_offset) ? {{14{offset[15]}},offset[15:0],2'b00}:
//                                                      {6'h0,instr_index[25:0]};

assign vid_dest_rd  = inst_addu | inst_subu | inst_slt | inst_sltu | inst_and |
                      inst_or   | inst_xor  | inst_nor | inst_sll  | inst_srl |
                      inst_sra  | inst_add  | inst_sub | inst_sllv | inst_srlv|
                      inst_srav | inst_mfhi | inst_mflo| inst_jalr;

assign vid_dest_rt  = inst_lui  | inst_addiu | inst_lw  | inst_addi | inst_slti | inst_sltiu |
                      inst_andi | inst_ori   | inst_xori| inst_lb   | inst_lbu  | inst_lh    |
                      inst_lhu  | inst_lwl   | inst_lwr | inst_mfc0;  

assign vid_dest_r31 = inst_jal | inst_bltzal | inst_bgezal;


assign vid_dest     = vid_dest_r31? 7'h1f:
                      vid_dest_rd ? {2'b00,rd[4:0]}:
                      vid_dest_rt ? {2'b00,rt[4:0]}:
                                    7'b1100000;
assign vid_baddr    = ds_baddr_r;
assign vid_bd       = bd;
//issuebus_o
assign issue_valid = vid_valid     ;
assign issue_baddr = vid_baddr     ;
assign issue_bd    = vid_bd        ;
assign issue_exc   = vid_exc       ;
assign issue_pc    = vid_pc        ;
assign issue_op    = vid_op        ;
assign issue_dest  = vid_dest      ;
assign issue_v1_en = v1_en         ;
assign issue_v1    = v1            ;
assign issue_v2_en = v2_en         ;
assign issue_v2    = v2            ;
assign issue_v3_en = v3_en         ;
assign issue_v3    = v3            ;


//control logic


// assign ds_cango       = !v_hazard || inc;
assign ds_cango       = ~v_hazard || (v_hazard && flush_taken) ||inc;
assign ds_allowin     = !ds_valid_r || ds_cango && es_allowin;
assign ds_to_es_valid = ds_valid_r && ds_cango;

always @(posedge clock) begin
    if(reset) begin
        ds_valid_r <= 1'b0;
    end else if(ds_allowin) begin
        if(flush_taken) ds_valid_r <= 1'b0;
        else ds_valid_r <= fs_to_ds_valid;
    end
    if(fs_to_ds_valid && ds_allowin) begin
        {ds_baddr_r,ds_exc_r,ds_pc_r,ds_inst_r} <= {fs_baddr,fs_exc,fs_pc,fs_inst};
    end
end

//exception detected
assign inst_cpu   = inst_ae    | inst_logic | inst_br | inst_io | inst_hl |
                    inst_shift | inst_trap  | inst_pri;
                  
assign vid_ex_ri  = ~inst_cpu;
assign vid_ex_sys = ds_valid_r & inst_syscall;
assign vid_ex_bp  = ds_valid_r & inst_break  ;

assign vid_exc    = {ds_exc_r[5:3],vid_ex_sys,vid_ex_bp,vid_ex_ri};


//bd
// always @(posedge clock) begin
//   if(reset) begin
//     bd <= 1'b0;
//   end else if(inst_br && ~inc && ds_valid_r) begin
//     bd <= 1'b1;
//   end else begin
//     bd <= 1'b0;
//   end
// end
always @(posedge clock) begin
  if(reset) begin
    bd <= 1'b0;
  end else if(flush_taken) begin
    bd <= 1'b0;
  end else if(inst_br&& ds_valid_r) begin
    bd <= 1'b1;
  end else begin
    bd <= 1'b0;
  end
end

//exc
assign exc = ds_valid_r & (vid_exc[5] | vid_exc[4] | vid_exc[3] | vid_exc[2] | vid_exc[1] | vid_exc[0] | state_exc | inst_eret);

assign inc = ds_valid_r && exc && (~state_kernel);



//branch & jump
assign rs_eq_rt = (v1 == v2);

assign rs_eq_z = (v1 == 32'h0); 

assign rs_lt_z = v1[31];

assign rs_ge_z = ~v1[31];


assign br_taken =            ((inst_beq &&  rs_eq_rt) ||
                             (inst_bne && !rs_eq_rt)  ||
                                             inst_jr  ||
                ((inst_bgez|inst_bgezal) && rs_ge_z)  ||
                  (inst_bgtz && rs_ge_z && !rs_eq_z)  ||
                 (inst_blez && (rs_lt_z || rs_eq_z))  ||
                ((inst_bltz|inst_bltzal) && rs_lt_z)  ||
                                              inst_j  ||
                                            inst_jalr ||
                                             inst_jal) && ds_valid_r;

assign br_target = (inst_beq    || inst_bne    || inst_bgez || 
                    inst_bltz   || inst_blez   || inst_bgtz ||
                    inst_bltzal || inst_bgezal                ) ? (fs_pc + {{14{offset[15]}},offset[15:0],2'b00}) :
                                         (inst_jr || inst_jalr) ? v3                                              :
                                           (inst_j || inst_jal) ? {fs_pc[31:28],instr_index[25:0],2'b0}           :
                                                                  32'h0;







endmodule



module vie_heap_2r1w_32x32(
  clock,

  ren0, raddr0, rvalue0,

  ren1, raddr1, rvalue1,

  wen0, waddr0, wvalue0

);
/* reg 0 is zero forever */
input           clock;

input           ren0;
input   [ 4:0]  raddr0;
output  [31:0]  rvalue0;

input           ren1;
input   [ 4:0]  raddr1;
output  [31:0]  rvalue1;

input           wen0;
input   [ 4:0]  waddr0;
input   [31:0]  wvalue0;

reg  [31:0]          heap_01, heap_02, heap_03, heap_04, heap_05, heap_06, heap_07;
reg  [31:0] heap_08, heap_09, heap_10, heap_11, heap_12, heap_13, heap_14, heap_15;
reg  [31:0] heap_16, heap_17, heap_18, heap_19, heap_20, heap_21, heap_22, heap_23;
reg  [31:0] heap_24, heap_25, heap_26, heap_27, heap_28, heap_29, heap_30, heap_31;

wire [31:0] raddr0_vec, raddr1_vec;
wire [31:0] heap_rvalue0, heap_rvalue1;
wire [31:0] waddr0_vec;
wire [31:0] wen0_vec;

/* Read */
v_decoder_5_32 u0_raddr_dec_5_32(.in({5{ren0}}&raddr0), .out(raddr0_vec));

v_decoder_5_32 u1_raddr_dec_5_32(.in({5{ren1}}&raddr1), .out(raddr1_vec));

assign heap_rvalue0 = {32{raddr0_vec[31]}}&heap_31 |
                      {32{raddr0_vec[30]}}&heap_30 |
                      {32{raddr0_vec[29]}}&heap_29 |
                      {32{raddr0_vec[28]}}&heap_28 |
                      {32{raddr0_vec[27]}}&heap_27 |
                      {32{raddr0_vec[26]}}&heap_26 |
                      {32{raddr0_vec[25]}}&heap_25 |
                      {32{raddr0_vec[24]}}&heap_24 |
                      {32{raddr0_vec[23]}}&heap_23 |
                      {32{raddr0_vec[22]}}&heap_22 |
                      {32{raddr0_vec[21]}}&heap_21 |
                      {32{raddr0_vec[20]}}&heap_20 |
                      {32{raddr0_vec[19]}}&heap_19 |
                      {32{raddr0_vec[18]}}&heap_18 |
                      {32{raddr0_vec[17]}}&heap_17 |
                      {32{raddr0_vec[16]}}&heap_16 |
                      {32{raddr0_vec[15]}}&heap_15 |
                      {32{raddr0_vec[14]}}&heap_14 |
                      {32{raddr0_vec[13]}}&heap_13 |
                      {32{raddr0_vec[12]}}&heap_12 |
                      {32{raddr0_vec[11]}}&heap_11 |
                      {32{raddr0_vec[10]}}&heap_10 |
                      {32{raddr0_vec[ 9]}}&heap_09 |
                      {32{raddr0_vec[ 8]}}&heap_08 |
                      {32{raddr0_vec[ 7]}}&heap_07 |
                      {32{raddr0_vec[ 6]}}&heap_06 |
                      {32{raddr0_vec[ 5]}}&heap_05 |
                      {32{raddr0_vec[ 4]}}&heap_04 |
                      {32{raddr0_vec[ 3]}}&heap_03 |
                      {32{raddr0_vec[ 2]}}&heap_02 |
                      {32{raddr0_vec[ 1]}}&heap_01 ;

assign heap_rvalue1 = {32{raddr1_vec[31]}}&heap_31 |
                      {32{raddr1_vec[30]}}&heap_30 |
                      {32{raddr1_vec[29]}}&heap_29 |
                      {32{raddr1_vec[28]}}&heap_28 |
                      {32{raddr1_vec[27]}}&heap_27 |
                      {32{raddr1_vec[26]}}&heap_26 |
                      {32{raddr1_vec[25]}}&heap_25 |
                      {32{raddr1_vec[24]}}&heap_24 |
                      {32{raddr1_vec[23]}}&heap_23 |
                      {32{raddr1_vec[22]}}&heap_22 |
                      {32{raddr1_vec[21]}}&heap_21 |
                      {32{raddr1_vec[20]}}&heap_20 |
                      {32{raddr1_vec[19]}}&heap_19 |
                      {32{raddr1_vec[18]}}&heap_18 |
                      {32{raddr1_vec[17]}}&heap_17 |
                      {32{raddr1_vec[16]}}&heap_16 |
                      {32{raddr1_vec[15]}}&heap_15 |
                      {32{raddr1_vec[14]}}&heap_14 |
                      {32{raddr1_vec[13]}}&heap_13 |
                      {32{raddr1_vec[12]}}&heap_12 |
                      {32{raddr1_vec[11]}}&heap_11 |
                      {32{raddr1_vec[10]}}&heap_10 |
                      {32{raddr1_vec[ 9]}}&heap_09 |
                      {32{raddr1_vec[ 8]}}&heap_08 |
                      {32{raddr1_vec[ 7]}}&heap_07 |
                      {32{raddr1_vec[ 6]}}&heap_06 |
                      {32{raddr1_vec[ 5]}}&heap_05 |
                      {32{raddr1_vec[ 4]}}&heap_04 |
                      {32{raddr1_vec[ 3]}}&heap_03 |
                      {32{raddr1_vec[ 2]}}&heap_02 |
                      {32{raddr1_vec[ 1]}}&heap_01 ;


assign rvalue0 = heap_rvalue0;

assign rvalue1 = heap_rvalue1;

/* Write */
v_decoder_5_32 u0_waddr_dec_5_32(.in(waddr0), .out(waddr0_vec));

assign wen0_vec = {32{wen0}}&waddr0_vec;

always @(posedge clock)
begin
  if (wen0_vec[31]) heap_31 <= wvalue0;
  if (wen0_vec[30]) heap_30 <= wvalue0;
  if (wen0_vec[29]) heap_29 <= wvalue0;
  if (wen0_vec[28]) heap_28 <= wvalue0;
  if (wen0_vec[27]) heap_27 <= wvalue0;
  if (wen0_vec[26]) heap_26 <= wvalue0;
  if (wen0_vec[25]) heap_25 <= wvalue0;
  if (wen0_vec[24]) heap_24 <= wvalue0;
  if (wen0_vec[23]) heap_23 <= wvalue0;
  if (wen0_vec[22]) heap_22 <= wvalue0;
  if (wen0_vec[21]) heap_21 <= wvalue0;
  if (wen0_vec[20]) heap_20 <= wvalue0;
  if (wen0_vec[19]) heap_19 <= wvalue0;
  if (wen0_vec[18]) heap_18 <= wvalue0;
  if (wen0_vec[17]) heap_17 <= wvalue0;
  if (wen0_vec[16]) heap_16 <= wvalue0;
  if (wen0_vec[15]) heap_15 <= wvalue0;
  if (wen0_vec[14]) heap_14 <= wvalue0;
  if (wen0_vec[13]) heap_13 <= wvalue0;
  if (wen0_vec[12]) heap_12 <= wvalue0;
  if (wen0_vec[11]) heap_11 <= wvalue0;
  if (wen0_vec[10]) heap_10 <= wvalue0;
  if (wen0_vec[ 9]) heap_09 <= wvalue0;
  if (wen0_vec[ 8]) heap_08 <= wvalue0;
  if (wen0_vec[ 7]) heap_07 <= wvalue0;
  if (wen0_vec[ 6]) heap_06 <= wvalue0;
  if (wen0_vec[ 5]) heap_05 <= wvalue0;
  if (wen0_vec[ 4]) heap_04 <= wvalue0;
  if (wen0_vec[ 3]) heap_03 <= wvalue0;
  if (wen0_vec[ 2]) heap_02 <= wvalue0;
  if (wen0_vec[ 1]) heap_01 <= wvalue0;
end

endmodule //vie_heap_2r1w_32x32