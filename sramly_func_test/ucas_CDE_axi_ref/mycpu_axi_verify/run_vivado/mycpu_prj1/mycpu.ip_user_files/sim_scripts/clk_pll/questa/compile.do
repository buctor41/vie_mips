vlib questa_lib/work
vlib questa_lib/msim

vlib questa_lib/msim/xpm
vlib questa_lib/msim/xil_defaultlib

vmap xpm questa_lib/msim/xpm
vmap xil_defaultlib questa_lib/msim/xil_defaultlib

vlog -work xpm -64 -sv "+incdir+../../../ipstatic" \
"E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
"E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -64 -93 \
"E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib -64 "+incdir+../../../ipstatic" \
"../../../../../../../../CPU_CDE_SRAM/mycpu_sram_verify/rtl/xilinx_ip/clk_pll/clk_pll_clk_wiz.v" \
"../../../../../../../../CPU_CDE_SRAM/mycpu_sram_verify/rtl/xilinx_ip/clk_pll/clk_pll.v" \

vlog -work xil_defaultlib \
"glbl.v"

