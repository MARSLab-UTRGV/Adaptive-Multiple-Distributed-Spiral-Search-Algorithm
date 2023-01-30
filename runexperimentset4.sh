#!/bin/bash
for i in 10 12 14 16 18 20
do
    ./runntimes.sh $1 experiments/DDSAU60minArea${i}x${i}.xml results/DDSAU60minArea${i}x${i}.txt
done
