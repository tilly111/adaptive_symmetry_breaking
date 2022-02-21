#!/bin/bash
tmp_counter=0
com_range_counter=1
for j in `seq 0 109`;
    do
      conf=ASB_experiment_$((11 + ${tmp_counter} % 11)).kconf
      n=3

      if ((${tmp_counter}  == 0)); then
        MAX_COMMUNICATION_RANGE=2
      elif ((${tmp_counter} == 11)); then
        MAX_COMMUNICATION_RANGE=5
      elif ((${tmp_counter} == 22)); then
        MAX_COMMUNICATION_RANGE=10
      elif ((${tmp_counter} == 33)); then
        MAX_COMMUNICATION_RANGE=15
      elif ((${tmp_counter} == 44)); then
        MAX_COMMUNICATION_RANGE=20
      elif ((${tmp_counter} == 55)); then
        MAX_COMMUNICATION_RANGE=25
      elif ((${tmp_counter} == 66)); then
        MAX_COMMUNICATION_RANGE=30
      elif ((${tmp_counter} == 77)); then
        MAX_COMMUNICATION_RANGE=35
      elif ((${tmp_counter} == 88)); then
        MAX_COMMUNICATION_RANGE=40
      elif ((${tmp_counter} == 99)); then
        MAX_COMMUNICATION_RANGE=45
    fi
    INITIAL_COMMUNICATION_RANGE=1
    EXP_NAME=experiment_adaptation_${j}_comrng_${INITIAL_COMMUNICATION_RANGE}

      tmp_counter=$(( ${tmp_counter} + 1 ))
      echo ${EXP_NAME} ${conf} ${MAX_COMMUNICATION_RANGE}
    done