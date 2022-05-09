#!/usr/bin/bash

# ************** Adjust settings there **************
title="DagPerformance"
MaxTaskNumber=10
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************
cd $ROOT_PATH/release
make -j8

for (( jobNumber=3; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	for util in $(seq 0.1 0.1 0.7)
	do
	totalTaskSetNumber=$(echo $util*150 | bc)
	echo ${totalTaskSetNumber%.*}
	./tests/GenerateTaskSet --taskSetNumber 2 --taskType 2 --schedulabilityCheck 2 --clearBeforeAdd 0 --deadlineType 0 --totalUtilization $util --N $jobNumber 
	done
done
