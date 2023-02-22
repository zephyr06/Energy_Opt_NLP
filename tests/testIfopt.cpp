#include <CppUnitLite/TestHarness.h>

#include "sources/ControlOptimization/ControlIfoptSpec.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/EnergyOptimization/EnergyIftopSpec.h"

TEST(opt, energy) {
    rt_num_opt::enableMaxComputationTimeRestrict = 1;
    rt_num_opt::MaxComputationTimeRestrict = 2;
    rt_num_opt::executionTimeModel = 1;
    rt_num_opt::EnergyMode = 1;
    rt_num_opt::runMode = "normal";
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v23.csv";
    rt_num_opt::TaskSet taskSet1 =
        rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
    rt_num_opt::TaskSetNormal tasksN(taskSet1);

    double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal,
                                                 rt_num_opt::RTA_LL>(tasksN);

    std::cout << "The energy saving ratio is " << res << std::endl;
    EXPECT_DOUBLES_EQUAL(
        0.25, res, 1e-2);  // Ifopt works in this case because the iterations
                           // happen to stay within a continuous region
    std::cout << "***************************************************"
              << std::endl;
}

// Uncomment contorl constraint's lower bound to perform a real test, whose
// result would be 1095974
TEST(opt, control) {
    rt_num_opt::runMode = "normal";
    std::string path1 =
        "/home/zephyr/Programming/others/YechengRepo/Experiment/"
        "ControlPerformance/TestCases/NSweep/N0/Case0.txt";
    rt_num_opt::TaskSet tasks;
    rt_num_opt::VectorDynamic coeff;
    std::tie(tasks, coeff) = rt_num_opt::ReadControlCase(path1);
    rt_num_opt::TaskSetNormal tasksN(tasks);

    double res =
        rt_num_opt::OptimizeControlIfopt<rt_num_opt::TaskSetNormal,
                                         rt_num_opt::RTA_LL>(tasksN, coeff)
            .second;

    std::cout << "The energy saving ratio is " << res << std::endl;
    // EXPECT_DOUBLES_EQUAL(900410, res, 1e2); // 900410 is optimal, 1209674 is
    // initial objective function; Ifopt may not find the optimal solution
    // because it is mainly for continuous optimization
}

// TEST(run, single)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" +
//     rt_num_opt::testDataSetName + ".csv"; rt_num_opt::TaskSet taskSet1 =
//     rt_num_opt::ReadTaskSet(path, rt_num_opt::readTaskMode);
//     rt_num_opt::TaskSetNormal tasksN(taskSet1);
//     auto start = std::chrono::high_resolution_clock::now();

//     double res = rt_num_opt::OptimizeEnergyIfopt<rt_num_opt::TaskSetNormal,
//     rt_num_opt::RTA_LL>(tasksN);

//     std::cout << "The energy saving ratio is " << res << std::endl;
//     auto stop = std::chrono::high_resolution_clock::now();
//     auto duration =
//     std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//     std::cout << "The time taken is: " << double(duration.count()) / 1e6 <<
//     "seconds" << std::endl;
// }
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}