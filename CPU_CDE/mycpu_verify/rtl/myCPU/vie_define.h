`include "vtools.v"
/****** Bus Width *********/
`define Vbrbus                           33
`define Vfsbus                           65
`define Vfromifcbus                      32
`define Vtoifcbus                        69
`define Vissuebus                        147
`define Vwsbus                           38
`define Vrsbus                           73
`define Vmsbus                           72
`define Vdebugbus                        73
`define Vrstatus                         41
`define Vmstatus                         40

/****** internal op*******/
`define VIE_OP_MOV      8'h05 //LUI
`define VIE_OP_SLL      8'h11 //NOP,SLLV
`define VIE_OP_SRL      8'h12
`define VIE_OP_SRA      8'h13

`define VIE_OP_ADDU     8'h19 //ADDIU
`define VIE_OP_SUBU     8'h1b 
`define VIE_OP_AND      8'h1c
`define VIE_OP_OR       8'h1d
`define VIE_OP_XOR      8'h1e
`define VIE_OP_NOR      8'h1f

`define VIE_OP_SLT      8'h26 //SLTI
`define VIE_OP_SLTU     8'h27 //SLTIU

`define VIE_OP_JR       8'h2d
`define VIE_OP_JAL      8'h2e

`define VIE_OP_BEQ      8'h30
`define VIE_OP_BNE      8'h31


`define VIE_OP_LW       8'h92
`define VIE_OP_SW       8'h9a

