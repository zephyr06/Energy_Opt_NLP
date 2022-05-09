#!/usr/bin/bash

# ************** Adjust settings there **************
title="DagPerformance"
MaxTaskNumber=10
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************
cd $ROOT_PATH/release
make -j8

for (( jobNumber=10; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	#for util in $(seq 0.1 0.1 0.8)
	#do
	#totalTaskSetNumber=$(echo $util*120 | bc)
	#echo ${totalTaskSetNumber%.*}
	./tests/GenerateTaskSet --taskSetNumber 100 --taskType 2 --schedulabilityCheck 2 --clearBeforeAdd 0 --deadlineType 0 --N $jobNumber --totalUtilization -1
	#done
done
