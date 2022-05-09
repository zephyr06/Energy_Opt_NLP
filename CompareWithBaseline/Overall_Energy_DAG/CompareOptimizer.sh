#!/usr/bin/bash

# ************** Adjust settings there **************
title="Overall_Energy_DAG"
MaxTaskNumber=7
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
	./tests/DAGBatch
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}


for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
	python $ROOT_PATH/CompareWithBaseline/ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# line search, no elimination, conjugate gradient descent
    python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 4
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization
	
	# dog-leg, eliminated, approximated Jacobian
    python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization
	
	# dog-leg, not eliminated, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization
	
	# GN, not eliminated, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 3
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization
	
	# LM, eliminated, approximated Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization
	
	# LM, eliminated, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization
	
	# LM, not eliminated, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "elimIte" --value 0
	perform_optimization
done

# visualize the result
python $ROOT_PATH/CompareWithBaseline/Visualize_performance.py  --minTaskNumber 5  --method_names "Dogleg" "LM" "GN" "cSD" --title $title  --maxTaskNumber $MaxTaskNumber
