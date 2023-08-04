#!/usr/bin/bash

# ************** Adjust settings there **************
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
title="ControlPerformance_Hybrid"
MaxTaskNumber=5
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
	
	# LM, without reorder
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "MaxLoopControl" --value 1000
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1

	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_exec_weight" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_obj_coeff_weight" --value 0
	perform_optimization $jobNumber
	
	# reorder with RM
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_exec_weight" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_obj_coeff_weight" --value 0
	perform_optimization $jobNumber

	# # reorder with RM+exec
	# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 1
	# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_exec_weight" --value 1
	# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_obj_coeff_weight" --value 0
	# perform_optimization $jobNumber

	# reorder with sorting
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_exec_weight" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "control_sort_obj_coeff_weight" --value -0.01
	perform_optimization $jobNumber	
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline

# show the comparison between NORTH and NORTH_RM
python draw_box_plot.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio" --exp_folder $title --main_data_index 1

# show the comparison between NORTH and NORTH_Sort
python draw_box_plot.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio" --exp_folder $title --main_data_index 2

cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
