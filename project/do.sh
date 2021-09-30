#!/bin/bash

pushd pid_controller
make
popd
#python3 optimize_pid_params.py
./run_main_pid.sh
