#!/usr/bin/bash

ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
cd $ROOT_PATH/release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

> FeasibleRatio.txt

for util in $(seq 0.1 0.1 0.9) # average per-core utilization
do
	./tests/GenerateTaskSet --taskSetNumber 500 --taskType 2 --schedulabilityCheck 0 --clearBeforeAdd 1 --deadlineType 0 --totalUtilization $util --N 3 
	make testEstimateFeasibleSolutionRatio.run
done
