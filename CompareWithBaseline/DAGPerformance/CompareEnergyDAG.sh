#!/usr/bin/bash

# ************** Adjust settings there **************
title="DAGPerformance"
MinTaskNumber=3
MaxTaskNumber=10
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
# python $ROOT_PATH/TaskData/ClearResFiles.py
python clear_result_files.py  --folder $title

python edit_yaml.py --entry "batchOptimizeFolder" --value $title
python edit_yaml.py --entry "core_m_dag" --value 4

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
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
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 16
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization $jobNumber
	
	# LM, eliminated, exact Jacobian
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization $jobNumber
	
	# LM, not eliminated, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization $jobNumber
	
	# SA
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 5
	perform_optimization $jobNumber
	
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"
