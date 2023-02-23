#!/usr/bin/bash

# ************** Adjust settings there **************
title="Optimizer"
MinTaskNumber=5
MaxTaskNumber=20
ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
# ***************************************************

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# clear buffer file content
cd $ROOT_PATH/CompareWithBaseline
python clear_result_files.py  --folder $title --Nmin $MinTaskNumber --Nmax $MaxTaskNumber

python edit_yaml.py --entry "batchOptimizeFolder" --value $title

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j8
	./tests/BatchControl $1
	cd $ROOT_PATH/CompareWithBaseline/Optimizer/EnergySaveRatio
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}


for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do

	echo "$title iteration is: $jobNumber"
	
	# LM, eliminated, approximated Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "MaxLoopControl" --value 100
	perform_optimization $jobNumber
	
	#L
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "setDiagonalDamping" --value 0
	perform_optimization $jobNumber
	
	# Dogleg
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 1
	perform_optimization $jobNumber
	
	# GN
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 3
	perform_optimization $jobNumber
	
	# cGD
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 4
	perform_optimization $jobNumber
	
	# **********************************************************************************	
	# LM, no elimination, exact Jacobian
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "exactJacobian" --value 0
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "jacobianScale" --value 1
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "MaxLoopControl" --value 1
	perform_optimization $jobNumber
	
	#L
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 2
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "setDiagonalDamping" --value 0
	perform_optimization $jobNumber
	
	# Dogleg
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 1
	perform_optimization $jobNumber
	
	# GN
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 3
	perform_optimization $jobNumber
	
	# cGD
    	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "optimizerType" --value 4
	perform_optimization $jobNumber
	
	
done



# visualize the result
cd $ROOT_PATH/CompareWithBaseline/$title
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber 5 --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"
