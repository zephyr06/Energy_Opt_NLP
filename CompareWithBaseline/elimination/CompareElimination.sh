#!/usr/bin/bash

title="elimination"
MaxTaskNumber=20

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

python clear_result_files.py --folder $title
python edit_yaml.py --entry "batchOptimizeFolder" --value $title


perform_optimization() {
	for (( jobNumber=5; jobNumber<=$MaxTaskNumber; jobNumber++ ))
	do
		cd ../CompareWithBaseline/$title
		echo "$title iteration is: $0, $jobNumber"
		# generate task set
	  	python ../ConvertYechengDataset.py --convertionNumber $jobNumber

		# Optimize energy consumption
		cd ../../release

		make -j4
		./tests/LLBatch1
		cd ../CompareWithBaseline
		sleep 1
	done
}


python edit_yaml.py --entry "elimIte" --value 0
python edit_yaml.py --entry "roundTypeInClamp" --value "rough"
perform_optimization 0

python edit_yaml.py --entry "elimIte" --value 1000
python edit_yaml.py --entry "roundTypeInClamp" --value "fine"
perform_optimization 1000

cd $title
# visualize the result
python ../Visualize_performance.py  --minTaskNumber 5 --method_names "NoElimination" "WithElimination" --title $title  --maxTaskNumber $MaxTaskNumber
