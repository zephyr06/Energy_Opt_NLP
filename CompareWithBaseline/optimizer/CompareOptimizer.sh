#!/usr/bin/bash

title="optimizer"
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
for optimizerType in {1..2}
do
	python edit_yaml.py --entry "optimizerType" --value $optimizerType
	for jobNumber in {5..7}
	do
		cd ../CompareWithBaseline/$title
		echo "$title iteration is: $optimizerType, $jobNumber"
		# generate task set
	  	python ../ConvertYechengDataset.py --convertionNumber $jobNumber

		# Optimize energy consumption
		cd ../../release

		make -j4
		./tests/LLBatch1
		cd ../CompareWithBaseline
		sleep 1
	done
done

cd $title
# visualize the result
python Visualize_performance.py  --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --optimizerNum 4
