#!/bin/bash
# usage : buildKernel.sh gui_config_option
# Use gui_config_option to specify how to configure kernel (e.g. menuconfig)
# Or run with no args to use default config

kernel_dir="$HOME/thesis/xlnx_sources/linux-adi"
source setupEnvironment
cd $kernel_dir 
make xilinx_zynq_defconfig
make $1
make uImage

