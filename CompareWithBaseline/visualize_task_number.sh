#!/usr/bin/bash

title="task_number"
MaxTaskNumber=20

# clear buffer file content
data_buffer_energy="data_buffer_energy_$title.txt"
> $data_buffer_energy
time_file="time_$title.txt"
> $time_file
dataset="../TaskData/$title"


for jobNumber in {5..6}
do
	echo "$title iteration is: $jobNumber"
	> ResultFiles/N$jobNumber.txt
	# generate task set

  python ConvertYechengDataset.py --convertionNumber $jobNumber
	# python ../sources/job_creator.py --num_tasksets $1 --num_job $jobNumber \
	# --deadline_portion 1 --utilization_total 0.5 --desperate_mode 0 --directory "$dataset"
	
	# modify TASK_NUMBER in sources/parameters.h
	# python Modify_task_number.py --task_number $jobNumber

	# Optimize energy consumption
	cd ../build

	make -j4
	./tests/LLCompare
	#./tests/WAPBatchCompare
	cd ../CompareWithBaseline
	sleep 1
done

# visualize the result
python Visualize_distribution.py
python Visualize_average_speed.py
