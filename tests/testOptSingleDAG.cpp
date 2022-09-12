#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
// TEST(OptimizeTaskSet, RTA_LL_V1)
// {
//     BeginTimer("main");
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".yaml";

//     rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
//     InitializeGlobalVector(tasksN.tasks_.size());
//     AssertBool(true, tasksN.tasks_.size() > 0, __LINE__);
//     auto start = std::chrono::high_resolution_clock::now();
//     double res = Energy_Opt<rt_num_opt::DAG_Nasri19, rt_num_opt::RTA_Nasri19>::OptimizeTaskSet(tasksN);
//     std::cout << blue << "The energy saving ratio is " << res << def << std::endl;
//     auto stop = std::chrono::high_resolution_clock::now();
//     auto duration = duration_cast<microseconds>(stop - start);
//     std::cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << std::endl;
//     EndTimer("main");
//     PrintTimer();
// }
TEST(OptimizeTaskSet, RTA_DAG_V1)
{
    BeginTimer("main");
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".yaml";

    rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
    InitializeGlobalVector(tasksN.tasks_.size());
    AssertBool(true, tasksN.tasks_.size() > 0, __LINE__);
    auto start = std::chrono::high_resolution_clock::now();
    // double res = Energy_Opt<rt_num_opt::DAG_Nasri19, rt_num_opt::RTA_Nasri19>::OptimizeTaskSet(tasksN);
    double res = OptimizeTaskSetIterative<FactorGraphEnergyNasri, rt_num_opt::DAG_Nasri19>(tasksN).second;
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
