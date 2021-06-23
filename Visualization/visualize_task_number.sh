#!/usr/bin/bash

title="task_number"

# clear buffer file content
data_buffer_energy="data_buffer_energy_$title.txt"
> $data_buffer_energy
time_file="time_$title.txt"
> $time_file
dataset="../TaskData/$title"


for jobNumber in {3..10}
do
	echo "$title iteration is: $jobNumber"
	# generate task set
	python ../sources/job_creator.py --num_tasksets $1 --num_job $jobNumber \
	--deadline_portion 1 --utilization_total 0.5 --desperate_mode 0 --directory "$dataset"
	
	# modify TASK_NUMBER in sources/parameters.h
	

	# Optimize energy consumption
	make ../build/testBatch.run
done

# visualize the result
schedulability_py="schedulability_$title.py"
python $schedulability_py --x_label "Number of tasks" --y_label "Schedulable task sets" \
--type "schedulability" --title "$title"
python $schedulability_py --x_label "Number of tasks" --y_label "Energy saving ratio" \
--type "energy" --title "$title"
