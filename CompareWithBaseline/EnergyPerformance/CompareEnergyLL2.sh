#!/usr/bin/bash

# ************** Adjust settings there **************
title="EnergyPerformance"
MaxTaskNumber=30
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
# python clear_result_files.py  --folder $title

python edit_yaml.py --entry "batchOptimizeFolder" --value $title

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j8
	./tests/LLBatch
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}


for (( jobNumber=40; jobNumber<=80; jobNumber+=10 ))
do
	# generate task set
	python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# LM, eliminated, approximated Jacobian
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization
	
	# LM, not eliminated, approximated Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization
	
	# IPM
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 6
	perform_optimization
	
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance_time.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance_time.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"
