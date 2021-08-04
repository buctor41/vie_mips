module bridge2x1(
    input            clk,
    input            resetn,

    input            inst_sram_req  ,
    input            inst_sram_wr   ,
    input  [ 1:0]    inst_sram_size ,
    input  [ 3:0]    inst_sram_wstrb,
    input  [31:0]    inst_sram_addr ,
    input  [31:0]    inst_sram_wdata,
    output           inst_sram_addr_ok,
    output           inst_sram_data_ok,
    output [31:0]    inst_sram_rdata,

    input            data_sram_req  ,
    input            data_sram_wr   ,
    input  [ 1:0]    data_sram_size ,
    input  [ 3:0]    data_sram_wstrb,
    input  [31:0]    data_sram_addr ,
    input  [31:0]    data_sram_wdata,
    output           data_sram_addr_ok,
    output           data_sram_data_ok,
    output [31:0]    data_sram_rdata,

    input  [31:0]    cpu_wb_pc,
    input  [ 3:0]    cpu_wb_rf_wen,
    input  [ 4:0]    cpu_wb_rf_wnum,
    input  [31:0]    cpu_wb_rf_wdata,

    output [ 3:0]    arid,
    output [31:0]    araddr,
    output [ 7:0]    arlen,
    output [ 2:0]    arsize,
    output [ 1:0]    arburst,
    output [ 1:0]    arlock,
    output [ 3:0]    arcache,
    output [ 2:0]    arprot,
    output           arvalid,
    input            arready,

    input  [ 3:0]    rid,
    input  [31:0]    rdata,
    input  [ 1:0]    rresp,
    input            rlast,
    input            rvalid,
    output           rready,

    output [ 3:0]    awid,
    output [31:0]    awaddr,
    output [ 7:0]    awlen,
    output [ 2:0]    awsize,
    output [ 1:0]    awburst,
    output [ 1:0]    awlock,
    output [ 3:0]    awcache,
    output [ 2:0]    awprot,
    output           awvalid,
    input            awready,

    output [ 3:0]    wid,
    output [31:0]    wdata,
    output [ 3:0]    wstrb,
    output           wlast,
    output           wvalid,
    input            wready,

    input  [ 3:0]    bid,
    input  [ 1:0]    bresp,
    input            bvalid,
    output           bready,

    // trace debug interface


    output [31:0]    debug_wb_pc,
    output [ 3:0]    debug_wb_rf_wen,
    output [ 4:0]    debug_wb_rf_wnum,
    output [31:0]    debug_wb_rf_wdata        
);

//read req
reg  [ 3:0]  arid_r   ;
reg  [31:0]  araddr_r ;
reg  [ 2:0]  arsize_r ;
reg          arvalid_r;

//read response
reg  [31:0]  rdata_r  ;
reg          rready_r ;

//write addr req
reg  [31:0]  awaddr_r ;
reg  [ 2:0]  awsize_r ;
reg          awvalid_r;

//write data req
reg   [31:0] wdata_r  ;
reg          wvalid_r ;
reg   [ 3:0] wstrb_r  ;

//write response
reg          bready_r;


//sram
reg          inst_sram_addr_ok_r;
reg          inst_sram_data_ok_r;
reg  [31:0]  inst_sram_rdata_r  ;

reg          data_sram_addr_ok_rr;
reg          data_sram_addr_ok_wr;
reg          data_sram_data_ok_rr;
reg          data_sram_data_ok_wr;
reg  [31:0]  data_sram_rdata_r  ;


//fsm0   read req
reg  [2:0] fsm_rr;

always @(posedge clk) begin
    if(~resetn) begin
        fsm_rr               <= 3'b001;
        inst_sram_addr_ok_r  <= 1'b0;
        data_sram_addr_ok_rr <= 1'b0;
        arvalid_r            <= 1'b0;
    end else if(fsm_rr == 3'b001 && data_sram_req && ~data_sram_wr) begin
        arid_r               <= 4'h1;
        araddr_r             <= data_sram_addr;
        arsize_r             <= {1'b0,data_sram_size};
        arvalid_r            <= 1'b1;
        fsm_rr               <= 3'b010;
    end else if(fsm_rr == 3'b001 && inst_sram_req && ~inst_sram_wr && ~(data_sram_req && ~data_sram_wr)) begin
        arid_r               <= 4'b0;
        araddr_r             <= inst_sram_addr;
        arsize_r             <= {1'b0,inst_sram_size};
        arvalid_r            <= 1'b1;
        fsm_rr               <= 3'b10;
    end else if(fsm_rr == 3'b010 && arready) begin
        arvalid_r            <= 1'b0;
        if(arid == 4'b0) begin
            inst_sram_addr_ok_r  <= 1'b1;
        end else begin
            data_sram_addr_ok_rr <= 1'b1;
        end
        fsm_rr               <= 3'b100;
    end else if(fsm_rr == 3'b100) begin
        inst_sram_addr_ok_r  <= 1'b0;
        data_sram_addr_ok_rr <= 1'b0;
        fsm_rr               <= 3'b001;                            
    end 
end

//fsm1  r s
always @(posedge clk) begin
    if(~resetn) begin
        inst_sram_data_ok_r  <= 1'b0     ;
        data_sram_data_ok_rr <= 1'b0     ;   
    end else if(rvalid) begin           
        if(rid == 4'h0) begin           //fetch inst
            inst_sram_data_ok_r  <= 1'b1 ;
            data_sram_data_ok_rr <= 1'b0 ;
            inst_sram_rdata_r    <= rdata;
        end else begin
            inst_sram_data_ok_r  <= 1'b0 ;
            data_sram_data_ok_rr <= 1'b1 ;
            data_sram_rdata_r    <= rdata;            
        end
    end else begin
        inst_sram_data_ok_r  <= 1'b0     ;
        data_sram_data_ok_rr <= 1'b0     ;        
    end
end

//fsm2  wr;
reg [4:0] fsm_wr;

always @(posedge clk) begin
    if(~resetn) begin
        fsm_wr                  <= 5'b00001;
        awvalid_r               <= 1'b0;
        wvalid_r                <= 1'b0;
        data_sram_addr_ok_wr    <= 1'b0;
    end else if(fsm_wr == 5'b00001 && data_sram_req && data_sram_wr) begin
        awvalid_r               <= 1'b1;
        awsize_r                <= {1'b0,data_sram_size};
        awaddr_r                <= data_sram_addr;
        fsm_wr                  <= 5'b00010;
    end else if(fsm_wr == 5'b00010 && awready) begin
        awvalid_r               <= 1'b0;
        fsm_wr                  <= 5'b00100;
    end else if(fsm_wr == 5'b00100) begin
        wvalid_r                <= 1'b1;
        wdata_r                 <= data_sram_wdata;
        wstrb_r                 <= data_sram_wstrb;
        fsm_wr                  <= 5'b01000; 
    end else if(fsm_wr == 5'b01000 && wready) begin
        wvalid_r                <= 1'b0;
        data_sram_addr_ok_wr    <= 1'b1;
        fsm_wr                  <= 5'b10000;
    end else if(fsm_wr == 5'b10000) begin
        data_sram_addr_ok_wr    <= 1'b0;
        fsm_wr                  <= 5'b00001; 
    end
end

//fsm3
always @(posedge clk) begin
    if(~resetn) begin
        data_sram_data_ok_wr        <= 1'b0;
    end else if(bvalid) begin
        data_sram_data_ok_wr        <= 1'b1;
    end else begin
        data_sram_data_ok_wr        <= 1'b0;  
    end
end

//read req
assign arid         = arid_r   ;
assign araddr       = araddr_r ;
assign arlen        = 8'h00    ;
assign arsize       = arsize_r ;
assign arburst      = 2'h1     ;
assign arlock       = 2'h0     ;
assign arcache      = 4'h0     ;
assign arprot       = 3'h0     ;
assign arvalid      = arvalid_r;

//read response
assign rready       = 1'b1 ;

//write addr req
assign awid         = 4'h1     ;
assign awaddr       = awaddr_r ;
assign awlen        = 8'h00    ;
assign awsize       = awsize_r ;
assign awburst      = 2'b01    ;
assign awlock       = 2'h0     ;
assign awcache      = 4'h0     ;
assign awprot       = 3'h0     ;
assign awvalid      = awvalid_r;

//write data req
assign wid          = 4'h1     ;
assign wdata        = wdata_r  ;
assign wstrb        = wstrb_r  ;
assign wlast        = 1'b1     ;
assign wvalid       = wvalid_r ;

//write response
assign bready       = 1'b1 ;

//sram
assign inst_sram_addr_ok = inst_sram_addr_ok_r;
assign inst_sram_data_ok = inst_sram_data_ok_r;
assign inst_sram_rdata   = inst_sram_rdata_r  ;

assign data_sram_addr_ok = data_sram_addr_ok_rr || data_sram_addr_ok_wr;
assign data_sram_rdata   = data_sram_rdata_r  ;

assign data_sram_data_ok = data_sram_data_ok_rr || data_sram_data_ok_wr;

assign debug_wb_pc       = cpu_wb_pc;
assign debug_wb_rf_wen   = cpu_wb_rf_wen;
assign debug_wb_rf_wnum  = cpu_wb_rf_wnum;
assign debug_wb_rf_wdata = cpu_wb_rf_wdata;

endmodule