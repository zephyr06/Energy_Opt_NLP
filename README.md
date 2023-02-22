# Numerical Optimization for Real-time Systems
This repo provides implementation for paper 'A General and Scalable Method for Optimizing
Real-time Systems with Continuous Variables'.
Warning!!: the source code in this repo still needs some serious cleaning work, it is suggested to check it out later.

# Dependencies
- [CMake](https://cmake.org/download/)
- [Boost](https://www.boost.org/users/download/)
- [GTSAM](https://github.com/borglab/gtsam)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [cppUnitLite](https://github.com/anonymousUser666666/CppUnitLite)
- [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
- [Intel Threaded Building Blocks (TBB)](https://github.com/wjakob/tbb) The latest official TBB release removed some legacy code, so if possible, try to install this old version.
- [ifopt](https://github.com/ethz-adrl/ifopt)
- [Pandas](https://pandas.pydata.org/getting_started.html)(pip install pandas)
- [Seaborn](https://seaborn.pydata.org/installing.html) (pip install seaborn)
- Please let me know if some other packages are missing

- More about installing GTSAM
    - Sometimes the latest version may not work well, in that case, consider using the commit `7f19e3f43d5d38808b4f30617a187b253eb92b91`
    - The Eigen library installed in your system may cause some conflictions. In that case, please add `set (GTSAM_USE_SYSTEM_EIGEN TRUE)` to `CMakeLists.txt` 


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
./tests/testOptSingleDAG.run # Optimize a single task set with DVFS subject to [Nasri19](https://drops.dagstuhl.de/opus/volltexte/2019/10758/) RTA model
./tests/testPeriodFactorsOpt2.run # Control performance optimization for a single task set
```

To optimize for several task sets collectively, use the scripts provided in `CompareWithBaseline/*/*.sh`. However, these scripts require loading optimization results of [Zhao20](https://ieeexplore.ieee.org/document/9355563). If you want to fully reproduce experiment figures, please ask the authors for code access, run their code, and modify the reading path in this project accordingly(When running our this, it will report path not found and where to modify the path); however, the code provided is enough to re-produce experiments related to our paper. 


# Other things to notice before running
- The parameters that influence optimization process can be found in sources/parameters.yaml. If performing optimization doesn't give good result, you can adjust deltaInitialDogleg if using dogleg optimizer, or initialLambda if using LM optimizer. 
- A lot of unit tests can be found in `tests` foldeer so that you have a better idea about each functions.
- This project uses absolute path, so please replace `/home/zephyr/Programming/Energy_Opt_NLP` with `/YOUR/LOCAL/PATH/Energy_Opt_NLP`

# Please let me know if you have any issues about this project and open a new issue ticket.