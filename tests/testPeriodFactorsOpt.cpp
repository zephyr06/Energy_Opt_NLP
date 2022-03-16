#include "../sources/ControlOptimize.h"
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;

TEST(case1, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // auto sth = UnitOptimizationPeriod(tasks, coeff, maskForElimination);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    // periodInitial1 << 32.2896, 347.818, 434.834, 104.653, 333.08;
    // UpdateTaskSetPeriod(tasks, periodInitial1);
    // maskForElimination[1] = 1;
    // auto sth = OptimizeTaskSetIterativeWeight<FactorGraphForceManifold>(tasks, coeff, maskForElimination);
    auto sth2 = OptimizeTaskSetIterative<FactorGraphForceManifold>(tasks, coeff, maskForElimination);
    // UpdateTaskSetPeriod(tasks, sth.first);
    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false}, maskForElimination);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
