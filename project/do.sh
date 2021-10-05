#!/bin/bash

pushd pid_controller
make
BUILD_OK=$?
popd
if [ $BUILD_OK -eq 0 ]; then
	if [ "$1" == "run" ]; then
    	./run_main_pid.sh
    elif [ "$1" == "optimize" ]; then
		python3 optimize_pid_params.py
    else
    	echo Unknown or missing option \'`$1`\'. Select \'run\' or \'optimize\'.
    fi
else
   echo BUILD FAILED
fi
