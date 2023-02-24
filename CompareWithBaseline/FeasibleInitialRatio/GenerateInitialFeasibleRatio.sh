#!/usr/bin/bash

ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"
cd $ROOT_PATH/release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make testEstimateFeasibleSolutionRatio

cd $ROOT_PATH/CompareWithBaseline/FeasibleInitialRatio
> FeasibleRatio.txt

cd $ROOT_PATH/release
for util in $(seq 0.1 0.1 0.9) # average per-core utilization
do
	./tests/testEstimateFeasibleSolutionRatio --U $util
done
