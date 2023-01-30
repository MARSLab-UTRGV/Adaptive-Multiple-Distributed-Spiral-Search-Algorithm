#!/bin/bash

sensor_detection_rate_per_tick=(1 0.0629 0.0424 0.0302 0.0214 0.0146 0.0089 0.0042)
sensor_error_rate_per_second=(0 0125 025 0375 05 0625 075 0875)
destination_error_stdev=(0 0.2 0.4 0.6 0.8 1.0)
destination_error_stdev_names=(0 02 04 06 08 10)
best_mu=(1.7, 1.8, 1.9, 1.9, 2.1, 2.1, 2.3, 2.5)

# Iterate over destination error stdevs
for i in $(seq 0 5)
do
    # Iterate over sensor error rates
    for j in  $(seq 0 7)
    do
	sed -e "s/destination_noise_stdev/${destination_error_stdev[${i}]}/g;s/prob_target_detection/${sensor_detection_rate_per_tick[${j}]}/g" < experiments/DDSA_template.xml > experiments/SensorAndDestinationError/DDSAPL8SearchersSensorError${sensor_error_rate_per_second[${j}]}DestinationError${destination_error_stdev_names[${i}]}Area40x40.xml
    done
done

# Iterate over destination error stdevs
for i in $(seq 0 10)
do
    # Iterate over sensor error rates
    for j in  $(seq 0 7)
    do
	mkdir -p results/SensorAndDestinationError//${sensor_error_rate_per_second[${j}]}
	./runntimes.sh $1 experiments/SensorAndDestinationError/DDSAPL8SearchersSensorError${sensor_error_rate_per_second[${j}]}DestinationError${destination_error_stdev_names[${i}]}Area40x40.xml results/SensorAndDestinationError/${sensor_error_rate_per_second[${j}]}/${destination_error_stdev_names[${i}]}.csv
    done
done
