# Numerical Optimization for Real-time Systems
This repo provides implementation for paper 'A General and Scalable Method for Optimizing
Real-time Systems with Continuous Variables'.

# Dependencies
- [CMake](https://cmake.org/download/)
- [Boost](https://www.boost.org/users/download/)
- [GTSAM](https://github.com/borglab/gtsam)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [cppUnitLite](https://github.com/anonymousUser666666/CppUnitLite)
- OpenCV
- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) 
- Please let me know if some other packages are missing

# Build and Run
To build the C++ code:
```
cd Energy_Opt_NLP
mkdir build
cd build
cmake ..
make -j4
make check -j8 # optional, run all the unit tests
./tests/testOptSingle.run # Optimize a single task set with DVFS subject to LL RTA model
./tests/testOptSingleDAG.run # Optimize a single task set with DVFS subject to [Narsi19](https://drops.dagstuhl.de/opus/volltexte/2019/10758/) RTA model
./tests/testPeriodFactorsOpt2.run # Control performance optimization for a single task set
```

To optimize for several task sets collectively, use the scripts provided in `CompareWithBaseline/*/*.sh`. However, these scripts require loading optimization results of [Zhao20](https://ieeexplore.ieee.org/document/9355563). If you want to reproduce experiment figures, please ask the authors for code access; if you only want to run code, then you can remove the loading part and modify the plot script accordingly.

# Other things to notice before running
- The parameters that influence optimization process can be found in sources/parameters.yaml. If performing optimization doesn't give good result, you can adjust deltaInitialDogleg if using dogleg optimizer, or initialLambda if using LM optimizer. 
- A lot of unit tests can be found in `tests` foldeer so that you have a better idea about each functions.
- This project uses absolute path, so please replace `/home/zephyr/Programming/Energy_Opt_NLP` with `/YOUR/LOCAL/PATH/Energy_Opt_NLP`

