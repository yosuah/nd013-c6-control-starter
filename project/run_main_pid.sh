#!/bin/bash

./kill_carla.sh && sleep 3.0

# Run the client. Parameters: 6 numbers, [constant_speed, constant_speed_reference, normal], [staright, normal]
./pid_controller/pid_controller 0.1 0 0.0125 2.0 0.8 0.1 constant_speed_reference straight & # first set of plots
sleep 1.0
python3 simulatorAPI.py # --add_obstacles

python3 plot_pid.py

mv throttle_data.png plots/throttle_data_constant_speed_ref.png
mv steer_data.png plots/steer_data_straight.png

# Generate second set of plots

./kill_carla.sh && sleep 3.0

./pid_controller/pid_controller 0.1 0 0.0125 2.0 0.8 0.1 normal normal &
sleep 1.0
python3 simulatorAPI.py --add_obstacles

python3 plot_pid.py

mv throttle_data.png plots/throttle_data_planned.png
mv steer_data.png plots/steer_data_planned.png
