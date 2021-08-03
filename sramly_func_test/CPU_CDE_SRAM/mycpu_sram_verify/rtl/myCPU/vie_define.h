/****** Bus Width *********/
`define Vbrbus                           33
`define Vfsbus                           103
`define Vfromifcbus                      34
`define Vtoifcbus                        72
`define Vissuebus                        186
`define Vwsbus                           38
`define Vrsbus                           162
`define Vmsbus                           127
`define Vdebugbus                        73
`define Vrstatus                         42
`define Vmstatus                         41
`define Vflushbus                        33
`define Vwstate                          2
`define Vestate                          2
`define Vmstate                          2

/****** internal op*******/
`define VIE_OP_MOV      8'h05 //LUI
`define VIE_OP_MFHI     8'h0c
`define VIE_OP_MFLO     8'h0d
`define VIE_OP_MTHI     8'h0e
`define VIE_OP_MTLO     8'h0f

`define VIE_OP_SLL      8'h11 //NOP,SLLV
`define VIE_OP_SRL      8'h12
`define VIE_OP_SRA      8'h13
`define VIE_OP_MULT     8'h14
`define VIE_OP_MULTU    8'h15
`define VIE_OP_DIV      8'h16
`define VIE_OP_DIVU     8'h17
`define VIE_OP_ADD      8'h18 //ADDI
`define VIE_OP_ADDU     8'h19 //ADDIU
`define VIE_OP_SUB      8'h1a
`define VIE_OP_SUBU     8'h1b 
`define VIE_OP_AND      8'h1c
`define VIE_OP_OR       8'h1d
`define VIE_OP_XOR      8'h1e
`define VIE_OP_NOR      8'h1f

`define VIE_OP_SLT      8'h26 //SLTI
`define VIE_OP_SLTU     8'h27 //SLTIU

`define VIE_OP_J        8'h2c
`define VIE_OP_JR       8'h2d
`define VIE_OP_JAL      8'h2e
`define VIE_OP_JALR     8'h2f

`define VIE_OP_BEQ      8'h30
`define VIE_OP_BNE      8'h31
`define VIE_OP_BLEZ     8'h32
`define VIE_OP_BGTZ     8'h33
`define VIE_OP_BLTZ     8'h34
`define VIE_OP_BGEZ     8'h35
`define VIE_OP_BLTZAL   8'h36
`define VIE_OP_BGEZAL   8'h37

`define VIE_OP_ERET     8'h87
`define VIE_OP_MFC0     8'h8c
`define VIE_OP_MTC0     8'h8d
`define VIE_OP_LB       8'h90
`define VIE_OP_LH       8'h91
`define VIE_OP_LW       8'h92
`define VIE_OP_LBU      8'h94
`define VIE_OP_LHU      8'h95
`define VIE_OP_SB       8'h98
`define VIE_OP_SH       8'h99
`define VIE_OP_SW       8'h9a

`define VIE_OP_LWL      8'hb8
`define VIE_OP_LWR      8'hb9
`define VIE_OP_SWL      8'hba
`define VIE_OP_SWR      8'hbb

//Cp0
`define CR_BADVADDR     8'h40
`define CR_COUNT        8'h48
`define CR_COMPARE      8'h58
`define CR_STATUS       8'h60
`define CR_CAUSE        8'h68
`define CR_EPC          8'h70    

//SRAMLY_FS state_machine
`define F_START         5'b00000
`define F_JUDGE         5'b00001
`define F_READY         5'b00010
`define F_HANDSHAKE     5'b00100
`define F_INT           5'b01000
`define F_CANCEL        5'b10000


`define E_START         4'b0000
`define E_JUDGE         4'b0001
`define E_WAIT          4'b0010 
`define E_HANDSHAKE     4'b0100 


