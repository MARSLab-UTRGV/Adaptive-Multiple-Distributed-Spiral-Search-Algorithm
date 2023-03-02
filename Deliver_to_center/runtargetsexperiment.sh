#!/bin/bash

for i in 20 40 60 80 100 120 140 160 180 200 220 240 260
do
    ./runntimes.sh $1 experiments/DSAU60minTargets${i}.xml final_results/DSAU60minTargets${i}.txt
done
