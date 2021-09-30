#!/bin/bash

./kill_carla.sh

sleep 3.0

./pid_controller/pid_controller 0 0 0 0.1875 0.0 0.0875 &
sleep 1.0
python3 simulatorAPI.py
