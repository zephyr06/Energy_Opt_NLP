#include "sources/EnergyOptimization/EnergyOptimize.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
using namespace std;
// TEST(case1, v1)
// {
//     BeginTimer("main");
//     // weightEnergy = 1;
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";

//     TaskSet tasks = ReadTaskSet(path, readTaskMode);
//     std::vector<bool> maskForElimination(tasks.size(), false);
//     auto sth = EnergyOptimize::OptimizeTaskSetIterative<FactorGraphEnergyLL>(tasks);

//     cout << "Execution time after optimization is " << sth.first << endl;
//     cout << "Actual objective function is" << sth.second << endl;
//     EndTimer("main");
//     PrintTimer();
// }
TEST(case2, v1)
{
    BeginTimer("main");
    // weightEnergy = 1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";

    TaskSet tasks = ReadTaskSet(path, readTaskMode);
    std::vector<bool> maskForElimination(tasks.size(), false);
    auto sth = EnergyOptimize::OptimizeTaskSetIterative<FactorGraphEnergyLL>(tasks);

    cout << "Execution time after optimization is " << sth.first << endl;
    cout << "Actual objective function is" << sth.second << endl;
    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
