#!/bin/bash

ps axf | grep /opt/carla | grep -v grep | awk -F ' ' '{print $1}' | xargs kill -9

until SDL_VIDEODRIVER=offscreen /opt/carla-simulator/CarlaUE4.sh -opengl; do
    echo "Server 'carla' crashed with exit code $?.  Respawning.." >&2
    sleep 1
done
