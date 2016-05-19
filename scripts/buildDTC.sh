#!/bin/bash

# You must have flex and bison installed to use this!
# sudo apt-get install flex bison

source ./setupEnvironment
cd ../xlnx_sources/dtc
make
