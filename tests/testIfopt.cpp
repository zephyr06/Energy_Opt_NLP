#include <CppUnitLite/TestHarness.h>

#include "sources/EnergyOptimization/EnergyIftopSpec.h"

#include "sources/ControlOptimization/ControlIfoptSpec.h"
#include "sources/ControlOptimization/ReadControlCases.h"

TEST(opt, energy)
{
    rt_num_opt::enableMaxComputationTimeRestrict = 1;
    rt_num_opt::MaxComputationTimeRestrict = 2;
    rt_num_opt::executionTimeModel = 1;
    rt_num_opt::EnergyMode = 1;
    rt_num_opt::runMode = "normal";
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v23.csv";
    rt_num_opt::TaskSet taskSet1 = rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
    rt_num_opt::TaskSetNormal tasksN(taskSet1);

    double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal, rt_num_opt::RTA_LL>(tasksN);

    std::cout << "The energy saving ratio is " << res << std::endl;
    EXPECT_DOUBLES_EQUAL(0.25, res, 1e-2);
    std::cout << "***************************************************" << std::endl;
}

TEST(opt, control)
{
    rt_num_opt::runMode = "normal";
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/" + rt_num_opt::controlPath + ".txt";
    rt_num_opt::TaskSet tasks;
    rt_num_opt::VectorDynamic coeff;
    std::tie(tasks, coeff) = rt_num_opt::ReadControlCase(path1);
    rt_num_opt::TaskSetNormal tasksN(tasks);

    double res = rt_num_opt::OptimizeControlIfopt<rt_num_opt::TaskSetNormal, rt_num_opt::RTA_LL>(tasksN, coeff);

    std::cout << "The energy saving ratio is " << res << std::endl;
    EXPECT_DOUBLES_EQUAL(0.25, res, 1e-2);
}

// TEST(run, single)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + rt_num_opt::testDataSetName + ".csv";
//     rt_num_opt::TaskSet taskSet1 = rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
//     rt_num_opt::TaskSetNormal tasksN(taskSet1);
//     auto start = std::chrono::high_resolution_clock::now();

//     double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal, rt_num_opt::RTA_LL>(tasksN);

//     std::cout << "The energy saving ratio is " << res << std::endl;
//     auto stop = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//     std::cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << std::endl;
// }
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}