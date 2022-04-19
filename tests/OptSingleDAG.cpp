#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/RTA_LL.h"
#include "../sources/RTA_DAG.h"
#include "../sources/profilier.h"
using namespace std::chrono;

TEST(OptimizeTaskSet, RTA_LL_V1)
{
    BeginTimer("main");
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSetDAG tasksN = ReadDAG_Tasks(path, readTaskMode);
    InitializeGlobalVector(tasksN.tasks_.size());
    auto start = chrono::high_resolution_clock::now();
    double res = Energy_Opt<TaskSetDAG, RTA_DAG>::OptimizeTaskSet(tasksN);
    cout << blue << "The energy saving ratio is " << res << def << endl;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << endl;
    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
