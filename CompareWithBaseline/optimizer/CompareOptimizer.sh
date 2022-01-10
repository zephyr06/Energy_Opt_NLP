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

python clear_result_files.py
python edit_yaml.py --entry "batchCompareFolder" --value $title
for optimizerType in {1..2}
do
	python edit_yaml.py --entry "optimizerType" --value $optimizerType
	for jobNumber in {5..7}
	do
		cd ../CompareWithBaseline/$title
		echo "$title iteration is: $jobNumber"
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
#python Visualize_distribution.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --baseline "MUA-incremental" 
#python Visualize_average_speed.py --minTaskNumber 5 --baseline "$title" --ylim 1e2 --maxTaskNumber $MaxTaskNumber
