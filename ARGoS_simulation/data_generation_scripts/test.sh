#!/bin/bash
tmp_counter=0
INITIAL_COMMUNICATION_RANGE=1
for j in `seq 0 5`;
    do

      # initial communication range = communication range
    NUM_ROBOTS=$(((25 * (tmp_counter+1))))
    echo ${NUM_ROBOTS}  ${tmp_counter}

    tmp_counter=$(( ${tmp_counter} + 1 ))
    done