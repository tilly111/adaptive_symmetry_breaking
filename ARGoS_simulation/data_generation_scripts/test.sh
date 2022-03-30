#!/bin/bash
tmp_counter=0
INITIAL_COMMUNICATION_RANGE=1
for j in `seq 0 16`;
    do

      # initial communication range = communication range
    if (($((${tmp_counter} % 17)) == 1)); then
          INITIAL_COMMUNICATION_RANGE=2
    elif (($((${tmp_counter} % 17)) == 2)); then
          INITIAL_COMMUNICATION_RANGE=3
    elif (($((${tmp_counter} % 17)) == 3)); then
          INITIAL_COMMUNICATION_RANGE=4
    elif (($((${tmp_counter} % 17)) == 4)); then
          INITIAL_COMMUNICATION_RANGE=5
    elif (($((${tmp_counter} % 17)) == 5)); then
          INITIAL_COMMUNICATION_RANGE=6
    elif (($((${tmp_counter} % 17)) == 6)); then
          INITIAL_COMMUNICATION_RANGE=7
    elif (($((${tmp_counter} % 17)) == 7)); then
          INITIAL_COMMUNICATION_RANGE=8
    elif (($((${tmp_counter} % 17)) == 8)); then
          INITIAL_COMMUNICATION_RANGE=9
    elif (($((${tmp_counter} % 17)) == 9)); then
          INITIAL_COMMUNICATION_RANGE=10
    elif (($((${tmp_counter} % 17)) == 10)); then
          INITIAL_COMMUNICATION_RANGE=15
    elif (($((${tmp_counter} % 17)) == 11)); then
          INITIAL_COMMUNICATION_RANGE=20
    elif (($((${tmp_counter} % 17)) == 12)); then
          INITIAL_COMMUNICATION_RANGE=25
    elif (($((${tmp_counter} % 17)) == 13)); then
          INITIAL_COMMUNICATION_RANGE=30
    elif (($((${tmp_counter} % 17)) == 14)); then
          INITIAL_COMMUNICATION_RANGE=35
    elif (($((${tmp_counter} % 17)) == 15)); then
          INITIAL_COMMUNICATION_RANGE=40
    elif (($((${tmp_counter} % 17)) == 16)); then
          INITIAL_COMMUNICATION_RANGE=45
    fi
    echo ${INITIAL_COMMUNICATION_RANGE}  ${tmp_counter}

    tmp_counter=$(( ${tmp_counter} + 1 ))
    done