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
assign fsbus_o[69 :64]  = fs_exc; 
assign fsbus_o[63 :32]  = addr;
assign fsbus_o[31 : 0]  = fs_inst;

wire            inst_sram_req  ;
wire            inst_sram_wr   ;
wire [ 1:0]     inst_sram_size ;
wire [ 3:0]     inst_sram_wstrb;
wire [31:0]     inst_sram_addr ;
wire [31:0]     inst_sram_wdata;

assign inst_ifc_o[71:71]    = inst_sram_req  ;
assign inst_ifc_o[70:70]    = inst_sram_wr   ;
assign inst_ifc_o[69:68]    = inst_sram_size ;
assign inst_ifc_o[67:64]    = inst_sram_wstrb;
assign inst_ifc_o[63:32]    = inst_sram_addr ;
assign inst_ifc_o[31: 0]    = inst_sram_wdata;

wire        inst_sram_addr_ok     = ifc_inst_i[33:33];
wire        inst_sram_data_ok     = ifc_inst_i[32:32];
wire[31:0]  inst_sram_rdata       = ifc_inst_i[31: 0];



/*
 *  Internal Signals and registers
 */
//pre_if
wire cancel;

wire pre_if_cango;
wire to_fs_valid;
wire [31:0] seq_pc ;
wire [31:0] next_pc;

//if
reg  fs_valid_r;
wire fs_cango;
wire fs_allowin;

//fetch
reg br;
reg [31:0] slot1;
reg [31:0] slot2;

reg         req  ;
wire        wr   ;
wire [1: 0] size ;
wire [3: 0] wstrb;
reg  [31:0] addr ;
wire [31:0] wdata;

wire        addr_ok = inst_sram_addr_ok;
wire        data_ok = inst_sram_data_ok;
wire [31:0] rdata   = inst_sram_rdata  ;

//exc
wire exc_adel;

//pre-if
assign pre_if_cango = req & addr_ok ;
assign to_fs_valid  = ~reset & pre_if_cango;
// assign seq_pc       = fs_pc_r + 3'h4;
// assign next_pc      = flush_taken ? flush_target :
//                       br_taken    ? br_target    : seq_pc;
assign next_pc      = addr;
//if
assign fs_cango       = data_ok;
assign fs_allowin     = !fs_valid_r || fs_cango && ds_allowin;
assign fs_to_ds_valid = fs_valid_r && fs_cango && (f_state == `F_HANDSHAKE) && (~flush_taken);

always @(posedge clock) begin
    if(reset) begin
        fs_valid_r <= 1'b0;
    end else if(fs_allowin) begin
        fs_valid_r <= to_fs_valid;
    end

    if(reset) begin
        fs_pc_r <= 32'hbfbffffc;
    end else if(to_fs_valid && fs_allowin) begin
        fs_pc_r <= addr;
    end
end

assign exc_adel         = fs_to_ds_valid && (slot2[1:0]!=2'b00);
assign fs_exc           = {exc_adel,5'b00};
assign fs_baddr         = exc_adel ? slot2 : 32'h0;

reg [ 4:0] f_state;

assign cancel = ~ds_allowin;

always @(posedge clock) begin
    if(reset) begin
        req     <= 1'b0;
        f_state <= `F_START;
        slot1   <= 32'hbfc00004;
        slot2   <= 32'hbfc00000;
    end else if(f_state == `F_START) begin
        f_state <= `F_JUDGE;
    end else if(f_state == `F_JUDGE) begin
        if(flush_taken) begin
            addr    <= flush_target;
            slot2   <= flush_target;
            slot1   <= flush_target + 3'h4;
            req     <= 1'b1;
        end else if(~cancel) begin
            req     <= 1'b1;
            addr    <= slot2;
            if(br_taken) begin
                slot1 <= br_target;
                br    <= 1'b1;
            end else begin
                br    <= 1'b0; 
            end            
        end
        if(~cancel)  f_state <= `F_READY; 
    end else if(f_state == `F_READY && flush_taken) begin
        if(inst_sram_addr_ok) begin
            f_state <= `F_INT;
            req <= 1'b0;
        end else begin
            f_state <= `F_CANCEL;
        end
        slot2 <= flush_target;
        slot1 <= flush_target + 3'h4;
    end else if(f_state == `F_READY && inst_sram_addr_ok && ~flush_taken) begin
        req <= 1'b0;
        f_state <= `F_HANDSHAKE; 
    end else if(f_state == `F_HANDSHAKE && flush_taken) begin
        slot2 <= flush_target;
        slot1 <= flush_target + 3'h4;
        if(inst_sram_data_ok) begin
            f_state <= `F_JUDGE;
        end else begin
            f_state <= `F_INT;
        end
    end else if(f_state == `F_HANDSHAKE && inst_sram_data_ok && ~flush_taken) begin
        if(!cancel) begin
            if(br) begin
                slot2 <= slot1;
                slot1 <= slot1 + 3'h4;
            end else begin
                slot2 <= slot1;
                slot1 <= slot1 + 3'h4;
            end
        end
        f_state <= `F_JUDGE;        
    end else if(f_state == `F_CANCEL && inst_sram_addr_ok) begin
        req <= 1'b0;
        f_state <= `F_INT;
    end else if(f_state == `F_INT && inst_sram_data_ok) begin
        f_state <= `F_JUDGE;
    end
end





//state_change
// always @(posedge clock) begin
//     if(reset) begin
//         f_state <= `F_START;
//     end else if(f_state == `F_START) begin
//         f_state <= `F_JUDGE;
//     end else if(f_state == `F_JUDGE) begin
//         if(~cancel) f_state <= `F_READY;
//     end else if(f_state == `F_READY && flush_taken) begin
//         if(inst_sram_addr_ok) begin
//             f_state <= `F_INT;
//             req <= 1'b0;
//         end else begin
//             f_state <= `F_CANCEL;
//         end
//         slot2 <= flush_target;
//         slot1 <= flush_target + 3'h4;
//     end else if(f_state == `F_CANCEL && inst_sram_addr_ok == 1'b1) begin
//         req     <= 1'b0;
//         f_state <= `F_INT;
//     end else if(f_state == `F_READY && inst_sram_addr_ok == 1'b1 && ~flush_taken) begin
//         f_state <= `F_HANDSHAKE;
//     end else if(f_state == `F_HANDSHAKE && flush_taken) begin
//         if(inst_sram_data_ok) begin
//             f_state <= `F_JUDGE;
//         end else begin
//             f_state <= `F_INT;
//         end
//     end else if(f_state == `F_HANDSHAKE && inst_sram_data_ok && ~flush_taken) begin
//         f_state <= `F_JUDGE;
//     end else if(f_state == `F_INT && inst_sram_data_ok) begin
//         f_state <= `F_JUDGE;
//     end
// end 

// //signals change
// always @(posedge clock) begin
//     if(reset) begin
//         req   <= 1'b0;
//         slot2 <= 32'hbfc00000; 
//         slot1 <= 32'hbfc00004;
//     end else if(f_state == `F_JUDGE) begin
//         if(flush_taken) begin
//             addr  <= flush_target;
//             slot2 <= flush_target;
//             slot1 <= flush_target + 3'h4;
//             req   <= 1'b1;
//         end else if(~cancel) begin
//             req   <= 1'b1;
//             addr <= slot2;
//             if(br_taken) begin
//                 slot1 <= br_target;
//                 br    <= 1'b1;
//             end else begin
//                 br    <= 1'b0; 
//             end
//         end
//     end else if(f_state == `F_READY && inst_sram_addr_ok == 1'b1) begin
//         req <= 1'b0;
//     end else if(f_state == `F_HANDSHAKE && flush_taken) begin
//         slot2 <= flush_target;
//         slot1 <= flush_target + 3'h4;
//     end else if(f_state == `F_HANDSHAKE && inst_sram_data_ok && ~flush_taken) begin
//         if(!cancel) begin
//             if(br) begin
//                 slot2 <= slot1;
//                 slot1 <= slot1 + 3'h4;
//             end else begin
//                 slot2 <= slot1;
//                 slot1 <= slot1 + 3'h4;
//             end
//         end
//     end
// end

assign wr    = 1'b0;
assign size  = 2'b10;
assign wstrb = 4'b00;
assign wdata = 32'b0;


assign inst_sram_req    = req  ;
assign inst_sram_wr     = wr   ;
assign inst_sram_size   = size ;
assign inst_sram_wstrb  = wstrb;
assign inst_sram_addr   = addr ;
assign inst_sram_wdata  = wdata;

assign fs_inst          = rdata;

endmodule
