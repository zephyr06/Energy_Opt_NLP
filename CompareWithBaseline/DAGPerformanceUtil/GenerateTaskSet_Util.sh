#!/usr/bin/bash

# ************** Adjust settings there **************
title="DAGPerformanceUtil"
jobNumber=7
TaskSetNumber=1000
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************
cd $ROOT_PATH/release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

for util in $(seq 0.1 0.1 0.9)
do
totalTaskSetNumber=$(echo $util*$TaskSetNumber | bc)
echo ${totalTaskSetNumber%.*}
./tests/GenerateTaskSet --taskSetNumber ${totalTaskSetNumber%.*} --taskType 2 --schedulabilityCheck 2 --clearBeforeAdd 0 --deadlineType 0 --totalUtilization $util --N $jobNumber 
done

