#include "../sources/EnergyOptimize.h"
#include "../sources/profilier.h"

TEST(case1, v1)
{
    BeginTimer("main");
    // weightEnergy = 1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";

    TaskSet tasks = ReadTaskSet(path, readTaskMode);

    std::vector<bool> maskForElimination(tasks.size(), false);
    // VectorDynamic periodInitial1 = GenerateVectorDynamic(tasks.size());
    // // periodInitial1 << 815, 815, 815, 815, 591, 815, 815, 815, 815, 815, 815, 815, 815, 815, 815, 815, 204, 815, 815;
    // periodInitial1 << 68, 300, 300, 300, 300;
    // UpdateTaskSetPeriod(tasks, periodInitial1);
    // maskForElimination[1] = 1;
    // auto sth = OptimizeTaskSetIterativeWeight<FactorGraphEnergyLL>(tasks, maskForElimination);
    auto sth = OptimizeTaskSetIterative<FactorGraphEnergyLL>(tasks, maskForElimination);

    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false}, maskForElimination);
    cout << "Actual objective function is" << sth.second << endl;
    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
