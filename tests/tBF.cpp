#include <chrono>

#include <CppUnitLite/TestHarness.h>

#include "sources/BruteForceOptimize.h"
using namespace std::chrono;

TEST(OptimizeTaskSet, a2)
{
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, readTaskMode);
    auto start = chrono::high_resolution_clock::now();
    double res = OptimizeTaskSetBf3(taskSet1);
    cout << "The energy saving ratio is " << res << endl;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
