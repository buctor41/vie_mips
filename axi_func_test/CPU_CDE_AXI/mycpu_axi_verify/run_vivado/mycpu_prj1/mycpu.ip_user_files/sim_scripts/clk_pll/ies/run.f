-makelib ies_lib/xpm -sv \
  "E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
  "E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \
-endlib
-makelib ies_lib/xpm \
  "E:/vivado_exe/Vivado/2019.2/data/ip/xpm/xpm_VCOMP.vhd" \
-endlib
-makelib ies_lib/xil_defaultlib \
  "../../../../../../rtl/xilinx_ip/clk_pll/clk_pll_clk_wiz.v" \
  "../../../../../../rtl/xilinx_ip/clk_pll/clk_pll.v" \
-endlib
-makelib ies_lib/xil_defaultlib \
  glbl.v
-endlib

