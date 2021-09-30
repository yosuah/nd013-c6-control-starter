#!/bin/bash

ps axf | grep /opt/carla | grep -v grep | awk -F ' ' '{print $1}' | xargs kill -9
