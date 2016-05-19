#!/bin/bash

source setupEnvironment
printenv PATH | grep xlnx_sources
cd ../xlnx_sources/u-boot-xlnx
make zynq_zed_config
make
