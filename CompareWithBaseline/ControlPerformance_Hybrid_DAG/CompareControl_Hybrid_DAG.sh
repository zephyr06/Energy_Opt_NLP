#!/usr/bin/bash

# ************** Adjust settings there **************
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
title="ControlPerformance_Hybrid_DAG"
MinTaskNumber=3
MaxTaskNumber=15
# ***************************************************

# clear results saved in TaskData
# python ClearResFiles.py --pathDataset TaskData/ControlPerformance_Hybrid_DAG/
cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/TaskData/
# python ClearResFiles.py --pathDataset TaskData/ControlPerformance_Hybrid_DAG/
cd $ROOT_PATH/CompareWithBaseline
python clear_result_files.py  --folder $title --Nmin $MinTaskNumber --Nmax $MaxTaskNumber
python edit_yaml.py --entry "batchOptimizeFolder" --value $title

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	make -j8
	echo "$1 ***** "
	./tests/BatchControl_Nasri19 $1
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}

for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# no need to generate task set because the script directly read it
	# python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# LM, without reorder (NORTH)
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 0
	perform_optimization $jobNumber
	
	# LM, with reorder-gradient
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 1
	perform_optimization $jobNumber

	# LM, with reorder-RM
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 2
	perform_optimization $jobNumber

	# LM, with reorder-coeff
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 3
	perform_optimization $jobNumber

done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
# show the comparison between NORTH and NORTH+
python draw_box_plot.py --minTaskNumber $MinTaskNumber --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio" --main_data_index 1
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio_Average"

cd $ROOT_PATH/TaskData/$title
zip -r Res_$(date +"%Y_%m_%d_%I_%M_%p").zip N*
