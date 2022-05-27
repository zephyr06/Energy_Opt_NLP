#!/usr/bin/bash

# ************** Adjust settings there **************
title="DAGPerformance"
MinTaskNumber=3
MaxTaskNumber=10
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************


for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	python LoadSenDAGRecord.py --taskSize $jobNumber
	
done

