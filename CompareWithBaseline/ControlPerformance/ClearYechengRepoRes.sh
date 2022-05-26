#!/usr/bin/bash

# ************** Adjust settings there **************
title="ControlPerformance"
MaxTaskNumber=20
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

python edit_yaml.py --entry "batchOptimizeFolder" --value $title


for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# no need to generate task set because the script directly read it
	# python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	python $ROOT_PATH/CompareWithBaseline/ClearYechengRecordsInMyRepo.py --taskSize $jobNumber --application "Control"
	
	# Load both MUA and MILP results
	python $ROOT_PATH/CompareWithBaseline/LoadYechengRecordTo.py --application "Control" --taskSize $jobNumber 
done
