#!/bin/sh

# 
# Vivado(TM)
# runme.sh: a Vivado-generated Runs Script for UNIX
# Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
# 

echo "This script was generated under a different operating system."
echo "Please update the PATH and LD_LIBRARY_PATH variables below, before executing this script"
exit

if [ -z "$PATH" ]; then
  PATH=E:/vivado_exe/Vitis/2019.2/bin;E:/vivado_exe/Vivado/2019.2/ids_lite/ISE/bin/nt64;E:/vivado_exe/Vivado/2019.2/ids_lite/ISE/lib/nt64:E:/vivado_exe/Vivado/2019.2/bin
else
  PATH=E:/vivado_exe/Vitis/2019.2/bin;E:/vivado_exe/Vivado/2019.2/ids_lite/ISE/bin/nt64;E:/vivado_exe/Vivado/2019.2/ids_lite/ISE/lib/nt64:E:/vivado_exe/Vivado/2019.2/bin:$PATH
fi
export PATH

if [ -z "$LD_LIBRARY_PATH" ]; then
  LD_LIBRARY_PATH=
else
  LD_LIBRARY_PATH=:$LD_LIBRARY_PATH
fi
export LD_LIBRARY_PATH

HD_PWD='F:/loogson/vie_mips/axi_func_test/CPU_CDE_AXI/mycpu_axi_verify/run_vivado/mycpu_prj1/mycpu.runs/axi_ram_synth_1'
cd "$HD_PWD"

HD_LOG=runme.log
/bin/touch $HD_LOG

ISEStep="./ISEWrap.sh"
EAStep()
{
     $ISEStep $HD_LOG "$@" >> $HD_LOG 2>&1
     if [ $? -ne 0 ]
     then
         exit
     fi
}

EAStep vivado -log axi_ram.vds -m64 -product Vivado -mode batch -messageDb vivado.pb -notrace -source axi_ram.tcl
