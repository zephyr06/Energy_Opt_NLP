#!/usr/bin/bash

# ************** Adjust settings there **************
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
title="ControlPerformance_Hybrid_DAG"
MaxTaskNumber=3
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
python clear_result_files.py  --folder $title --Nmin 3 --Nmax $MaxTaskNumber
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

for (( jobNumber=3; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# no need to generate task set because the script directly read it
	# python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# LM, without reorder (NORTH)
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 0
	perform_optimization $jobNumber
	
	# LM, with reorder (NORTH+)
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "enableReorder" --value 1
	perform_optimization $jobNumber
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
# show the comparison between NORTH and NORTH+
python draw_box_plot.py --minTaskNumber 3 --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio" --main_data_index 1
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 3 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"

cd $ROOT_PATH/TaskData/$title
zip Res_$(date +"%Y_%m_%d_%I_%M_%p").zip N*
