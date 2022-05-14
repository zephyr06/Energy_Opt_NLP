#!/usr/bin/bash

# ************** Adjust settings there **************
title="DAGPerformanceUtil"
MinUtil=1
MaxUtil=9
MinTaskNumber=7
MaxTaskNumber=7
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
python $ROOT_PATH/TaskData/ClearResFiles.py
python clear_result_files.py  --folder $title

python edit_yaml.py --entry "batchOptimizeFolder" --value $title
python edit_yaml.py --entry "core_m_dag" --value 4

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j8
	./tests/DAGBatch $1
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}


for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
	
	echo "$title iteration is: $jobNumber"
	
	# LM, eliminated, approximated Jacobian
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization $jobNumber
	
	# LM, not eliminated, approximated Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization $jobNumber
	
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minUtil $MinUtil --title $title  --maxUtil $MaxUtil --source_task_index $MinTaskNumber --root $ROOT_PATH
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinUtil --title $title  --maxTaskNumber $MaxUtil --data_source "Time"
