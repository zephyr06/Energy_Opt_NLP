#include "../sources/EnergyOptimize.h"
#include "../sources/profilier.h"
using namespace rt_num_opt;
using namespace std;
TEST(case1, v1)
{
    BeginTimer("main");
    // weightEnergy = 1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";

    TaskSet tasks = ReadTaskSet(path, readTaskMode);

    // VectorDynamic periodInitial1 = GenerateVectorDynamic(tasks.size());
    // // periodInitial1 << 815, 815, 815, 815, 591, 815, 815, 815, 815, 815, 815, 815, 815, 815, 815, 815, 204, 815, 815;
    // periodInitial1 << 68, 300, 300, 300, 300;
    // UpdateTaskSetPeriod(tasks, periodInitial1);
    // maskForElimination[1] = 1;
    std::vector<bool> maskForElimination(tasks.size(), false);

    // auto sth = EnergyOptimize::UnitOptimizationPeriod<FactorGraphEnergyLL>(tasks);
    auto sth = EnergyOptimize::OptimizeTaskSetIterative<FactorGraphEnergyLL>(tasks);

    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false}, maskForElimination);
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
