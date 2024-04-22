#include <CppUnitLite/TestHarness.h>

#include <chrono>

#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/Utils/Parameters.h"
// #include "sources/RTA_WAP.h"
// #include "sources/Generate_WAP.h"
#include "sources/Tools/profilier.h"
using namespace std::chrono;
using namespace rt_num_opt;
using namespace std;

using Opt_LL = Energy_Opt<Task, RTA_LL>;
// using Opt_WAP = Energy_Opt<Task, RTA_WAP>;

TEST(OptimizeTaskSet, RTA_LL_V1) {
    BeginTimer("main");
    if (optimizerType >= 5)
        optimizerType = 2;
    string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n2_v3.csv";
    CoutWarning("Default path used!");
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" +
    //               testDataSetName + ".csv";
    try {
        TaskSet taskSet1 = ReadTaskSet(path, readTaskMode);
        TaskSetNormal tasksN(taskSet1);
        InitializeGlobalVector(taskSet1.size());
        auto start = chrono::high_resolution_clock::now();
        double res = Energy_Opt<TaskSetNormal, RTA_LL>::OptimizeTaskSet(tasksN);
        cout << blue << "The energy saving ratio is " << res << def << endl;
        auto stop = chrono::high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "The time taken is: " << double(duration.count()) / 1e6
             << "seconds" << endl;
    } catch (...) {
        CoutWarning("Error occurred!");
        return;
    }

    EndTimer("main");
    PrintTimer();
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
