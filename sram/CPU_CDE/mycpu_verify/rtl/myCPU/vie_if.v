module vie_if_stage(
    clock,
    reset,

    ds_allowin,
    brbus_i,
    fsbus_o,

    ifc_inst_i,
    inst_ifc_o,
    flushbus_i
);
input                               clock;
input                               reset;

input                               ds_allowin;
input  [`Vbrbus               -1:0] brbus_i;
output [`Vfsbus               -1:0] fsbus_o;

input  [`Vfromifcbus          -1:0] ifc_inst_i;
output [`Vtoifcbus            -1:0] inst_ifc_o;

input  [`Vflushbus            -1:0] flushbus_i;
//wire        br_taken  = brbus_i[64:64];
//wire [31:0] br_base   = brbus_i[63:32];
//wire [31:0] br_offset = brbus_i[31: 0];       
wire        br_taken  = brbus_i[32:32];
wire [31:0] br_target = brbus_i[31: 0];


wire        flush_taken  = flushbus_i[32:32];
wire [31:0] flush_target = flushbus_i[31: 0];

reg  [31:0] fs_pc_r;
wire [31:0] fs_inst;
wire [ 5:0] fs_exc ;
wire [31:0] fs_baddr;
wire        fs_to_ds_valid;

assign fsbus_o[102:102] = fs_to_ds_valid;
assign fsbus_o[101: 70] = fs_baddr      ;
assign fsbus_o[69 :64] = fs_exc; 
assign fsbus_o[63 :32] = fs_pc_r;
assign fsbus_o[31 : 0] = fs_inst;

assign fs_inst        = ifc_inst_i;

wire        inst_sram_en;
wire [3 :0] inst_sram_wen;
wire [31:0] inst_sram_addr;
wire [31:0] inst_sram_wdata;

assign inst_ifc_o[68:68] = inst_sram_en;
assign inst_ifc_o[67:64] = inst_sram_wen;
assign inst_ifc_o[63:32] = inst_sram_addr;
assign inst_ifc_o[31: 0] = inst_sram_wdata;

/*
 *  Internal Signals and registers
 */
reg  fs_valid_r;
wire fs_cango;
wire fs_allowin;
wire to_fs_valid;

wire exc_adel;
wire [31:0] seq_pc ;
wire [31:0] next_pc;





//pre-if
assign to_fs_valid  = ~reset;
assign seq_pc       = fs_pc_r + 3'h4;
assign next_pc      = flush_taken ? flush_target :
                      br_taken    ? br_target    : seq_pc;
//assign pc_adder_a   = br_taken ? br_base   : fs_pc_r;
//assign pc_adder_b   = br_taken ? br_offset : 32'h4  ;
//assign pc_adder_res = pc_adder_a + pc_adder_b;
//assign next_pc      = pc_adder_res;
//if
assign fs_cango       = 1'b1;
assign fs_allowin     = !fs_valid_r || fs_cango && ds_allowin;
assign fs_to_ds_valid = fs_valid_r && fs_cango;

always @(posedge clock) begin
    if(reset) begin
        fs_valid_r <= 1'b0;
    end else if(fs_allowin) begin
        fs_valid_r <= to_fs_valid;
    end

    if(reset) begin
        fs_pc_r <= 32'hbfbffffc;
    end else if(to_fs_valid && fs_allowin) begin
        fs_pc_r <= next_pc;
    end
end

assign exc_adel         = to_fs_valid && fs_allowin && (fs_pc_r[1:0]!=2'b00);
assign fs_exc           = {exc_adel,5'b00};
assign fs_baddr         = exc_adel ? fs_pc_r[31:0] : 32'h0;

assign inst_sram_en     = to_fs_valid && fs_allowin;
assign inst_sram_wen   = 4'h0;
assign inst_sram_addr  = next_pc;
assign inst_sram_wdata = 32'b0;


endmodule