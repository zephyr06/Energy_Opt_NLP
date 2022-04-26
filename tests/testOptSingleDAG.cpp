#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/RTA/RTA_DAG.h"
#include "sources/RTA/RTA_Fonseca2019.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
TEST(OptimizeTaskSet, RTA_LL_V1)
{
    BeginTimer("main");
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";

    DAG_Fonseca tasksN = ReadDAGFonseca_Tasks(path, readTaskMode);
    InitializeGlobalVector(tasksN.tasks_.size());
    AssertBool(true, tasksN.tasks_.size() > 0, __LINE__);
    auto start = std::chrono::high_resolution_clock::now();
    double res = Energy_Opt<DAG_Fonseca, RTA_Fonseca2019>::OptimizeTaskSet(tasksN);
    std::cout << blue << "The energy saving ratio is " << res << def << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << std::endl;
    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
