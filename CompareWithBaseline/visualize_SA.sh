#!/usr/bin/bash

title="task_number"
MaxTaskNumber=6

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

	# Optimize energy consumption
	cd ../build
	./tests/GenerateTaskSet --N $jobNumber --taskSetNumber 1000 --totalUtilization 0.5 --NumberOfProcessor 1 --periodMin 100 --periodMax 10000
	make -j4
#	./tests/LLCompare
	./tests/WAPCompare
	cd ../CompareWithBaseline
	sleep 1
done

# visualize the result
python Visualize_distribution.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --baseline "SA"
python Visualize_average_speed.py --minTaskNumber 5 --maxTaskNumber $MaxTaskNumber --baseline "SA" --ylim 1e0
