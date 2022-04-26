#!/usr/bin/bash

title="CompareSpeed"
MaxTaskNumber=7

# clear buffer file content
time_file="time_task_number.txt"
> $time_file

cp parameters.yaml ../../sources/parameters.yaml

cd ../..
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
cd ../CompareWithBaseline

python clear_result_files.py  --folder $title
python edit_yaml.py --entry "batchOptimizeFolder" --value $title

perform_optimization(){
	# Optimize energy consumption
	cd ../../release
	make -j4
	./tests/LLBatch1
	cd ../CompareWithBaseline
	sleep 1
}


for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
	python ../ConvertYechengDataset.py --convertionNumber $jobNumber
	echo "$title iteration is: $jobNumber"
	
	# dog-leg, eliminated, approximated Jacobian
	python edit_yaml.py --entry "optimizerType" --value 1
	python edit_yaml.py --entry "exactJacobian" --value 0
	python edit_yaml.py --entry "elimIte" --value 1000
	perform_optimization()
done

# visualize the result
python ../Visualize_performance.py  --minTaskNumber 5  --method_names "Dogleg" "LM" "GN" "cSD" --title $title  --maxTaskNumber $MaxTaskNumber
