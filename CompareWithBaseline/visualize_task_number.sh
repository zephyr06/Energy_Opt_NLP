#!/usr/bin/bash

title="task_number"
MaxTaskNumber=20

# clear buffer file content
data_buffer_energy="data_buffer_energy_$title.txt"
> $data_buffer_energy
time_file="time_$title.txt"
> $time_file
dataset="../TaskData/$title"


for jobNumber in {5..20}
do
	echo "$title iteration is: $jobNumber"
	> ResultFiles/N$jobNumber.txt
	# generate task set

  	python ConvertYechengDataset.py --convertionNumber $jobNumber

	# Optimize energy consumption
	cd ../release

	make -j4
	./tests/LLCompare
	#./tests/WAPBatchCompare
	cd ../CompareWithBaseline
	sleep 1
done

# visualize the result
python Visualize_distribution.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --baseline "MUA-incremental" 
python Visualize_average_speed.py --minTaskNumber 5 --baseline "MUA-incremental" --ylim 1e2 --maxTaskNumber $MaxTaskNumber
