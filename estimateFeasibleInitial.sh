cd release
make GenerateTaskSet
./tests/GenerateTaskSet --taskSetNumber 100 --N 3 --clearBeforeAdd 1 --totalUtilization 0.6 --schedulabilityCheck 0
make testEstimateFeasibleSolutionRatioSimple.run
