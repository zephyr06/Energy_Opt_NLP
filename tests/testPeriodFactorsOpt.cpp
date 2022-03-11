#include "../sources/ControlOptimize.h"
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
TEST(ExtractResults, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[1] = true;
    Values result;
    result.insert(GenerateControlKey(0, "period"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(2, "period"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(3, "period"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(4, "period"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(0, "response"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(1, "response"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(2, "response"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(3, "response"), GenerateVectorDynamic1D(1));
    result.insert(GenerateControlKey(4, "response"), GenerateVectorDynamic1D(1));
    VectorDynamic expectT = GenerateVectorDynamic(5);
    expectT = expectT.array() + 1;
    VectorDynamic expectR = expectT;
    expectT(1, 0) = tasks[1].period;
    AssertEigenEqualVector(expectT, ExtractResults(result, tasks).first);
    AssertEigenEqualVector(expectR, ExtractResults(result, tasks).second);
}

// TEST(FindEliminatedVariables, v1)
// {
//     noiseModelSigma = 1;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     std::vector<bool> maskForElimination(tasks.size(), false);

//     VectorDynamic initial = GenerateVectorDynamic(5);
//     initial << 45, 372.719, 454.248, 128.127, 358.683;
//     UpdateTaskSetPeriod(tasks, initial);
//     FindEliminatedVariables(tasks, maskForElimination, 1);
//     for (auto a : maskForElimination)
//         cout << a << ", ";
//     cout << endl;
// }
TEST(case1, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    // auto sth = UnitOptimizationPeriod(tasks, coeff, maskForElimination);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    // periodInitial1 << 32.2896, 347.818, 434.834, 104.653, 333.08;
    // UpdateTaskSetPeriod(tasks, periodInitial1);
    // maskForElimination[1] = 1;
    auto sth = OptimizeTaskSetIterativeWeight(tasks, coeff, maskForElimination);
    // auto sth = OptimizeTaskSetIterative(tasks, coeff, maskForElimination);
    // UpdateTaskSetPeriod(tasks, sth.first);
    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false}, maskForElimination);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
