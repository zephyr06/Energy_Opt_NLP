#include <chrono>

#include <CppUnitLite/TestHarness.h>

#include "../sources/Optimize.h"
using namespace std::chrono;
// TEST(UpdateTaskSetExecutionTime, a1)
// {
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";

//     TaskSet taskSet1 = ReadTaskSet(path, "RM");

//     ComputationTimeVector comp;
//     comp << 10, 20, 30;
//     UpdateTaskSetExecutionTime(taskSet1, comp);
//     CHECK_EQUAL(10, taskSet1[0].executionTime);
// }

TEST(ComputationFactor, a1)
{
    // global optimzal result: 143 20 185

    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    auto start = chrono::high_resolution_clock::now();
    auto res = OptimizeTaskSet(taskSet1);
    cout << "The energy saving ratio is " << res << endl;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "The time taken is: " << float(duration.count()) / 1e6 << "seconds" << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
