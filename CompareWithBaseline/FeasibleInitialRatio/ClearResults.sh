#!/usr/bin/bash

ROOT_PATH="/home/zephyr/Programming/Energy_Opt_NLP"

cd $ROOT_PATH/CompareWithBaseline/FeasibleInitialRatio
for i in $(seq 1 1 1) 
do
	cd U$i
	rm *Res*
done
