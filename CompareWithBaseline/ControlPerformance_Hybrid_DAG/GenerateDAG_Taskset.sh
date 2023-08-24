#!/usr/bin/bash

# ************** Adjust settings there **************
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
title="ControlPerformance_Hybrid_DAG"
MaxTaskNumber=3
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/release
cmake ..
make GenerateTaskSet -j2


for (( jobNumber=3; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	echo "$title iteration is: $jobNumber"	
	./tests/GenerateTaskSet --taskType 3 --N $jobNumber --taskSetNumber 100 --NumberOfProcessor 2 --directory $ROOT_PATH/TaskData/$title	
done
