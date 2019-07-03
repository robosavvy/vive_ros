#!/bin/bash

candidate_1=${HOME}/.steam/ubuntu12_32/steam-runtime/run.sh
candidate_2=${HOME}/.steam/steam/ubuntu12_32/steam-runtime/run.sh

if [ -e $candidate_1 ]; then
    echo "runtime setup script found on $candidate_1"
    exec $candidate_1 "$@"
    
elif [ -e $candidate_2 ]; then
    echo "runtime setup script found on $candidate_2"
    exec $candidate_2 "$@"

else
    echo "\e[31m [ERROR] steam runtime setup script not found !! \e[m"
    exit 1
fi
