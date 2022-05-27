#!/usr/bin/bash

# ************** Adjust settings there **************
title="ControlPerformance"
MaxTaskNumber=30
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************


for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# no need to generate task set because the script directly read it
	# python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	python $ROOT_PATH/CompareWithBaseline/ClearYechengRecordsInMyRepo.py --taskSize $jobNumber --application "Energy"
	
	# Load both MUA and MILP results
	python $ROOT_PATH/CompareWithBaseline/LoadYechengRecordTo.py --application "Energy" --taskSize $jobNumber 
done
