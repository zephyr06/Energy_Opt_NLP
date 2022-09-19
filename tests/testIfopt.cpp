#include <CppUnitLite/TestHarness.h>

#include "sources/EnergyOptimization/EnergyIftopSpec.h"

TEST(opt, energy)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + rt_num_opt::testDataSetName + ".csv";
    rt_num_opt::TaskSet taskSet1 = rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
    rt_num_opt::TaskSetNormal tasksN(taskSet1);
    auto start = std::chrono::high_resolution_clock::now();

    double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal, rt_num_opt::RTA_LL>(tasksN);

    std::cout << "The energy saving ratio is " << res << std::endl;
    EXPECT_DOUBLES_EQUAL(0.5, res, 1e-1);
}

TEST(run, single)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + rt_num_opt::testDataSetName + ".csv";
    rt_num_opt::TaskSet taskSet1 = rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
    rt_num_opt::TaskSetNormal tasksN(taskSet1);
    auto start = std::chrono::high_resolution_clock::now();

    double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal, rt_num_opt::RTA_LL>(tasksN);

    std::cout << "The energy saving ratio is " << res << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << std::endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}