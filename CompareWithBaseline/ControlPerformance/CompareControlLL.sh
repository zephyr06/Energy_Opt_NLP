#!/usr/bin/bash

# ************** Adjust settings there **************
title="ControlPerformance"
MaxTaskNumber=20
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
python clear_result_files.py  --folder $title
python edit_yaml.py --entry "batchOptimizeFolder" --value $title

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	make -j8
	echo "$1 ***** "
	./tests/BatchControl $1
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}

for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# no need to generate task set because the script directly read it
	# python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# LM, eliminated
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "MaxLoopControl" --value 1000
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	perform_optimization $jobNumber
	
	# LM, NMBO
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "MaxLoopControl" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	perform_optimization $jobNumber
	
	# IPM
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 6
	perform_optimization $jobNumber
	
	# Load both MUA and MILP results
	python $ROOT_PATH/CompareWithBaseline/LoadYechengRecordTo.py --application "Control" --taskSize $jobNumber 
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"
python $ROOT_PATH/CompareWithBaseline/$title/draw_box_plot.py
