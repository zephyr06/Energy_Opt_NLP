#include <CppUnitLite/TestHarness.h>
#include "../sources/Optimize.h"
#include "../sources/testMy.h"
TEST(evaluateError, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    int N = tasks.size();
    InitializeGlobalVector(tasks.size());
    int lastTaskDoNotNeedOptimize = -1;
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma((N - lastTaskDoNotNeedOptimize - 1), noiseModelSigma);
    Energy_Opt<RTA_LL>::ComputationFactor factor(key, tasks, -1, responseTimeInitial, model);
    VectorDynamic x = GetParameterVD<double>(tasks, "executionTime");
    executionTimeModel = 1;
    EnergyMode = 1;
    VectorDynamic expect = x;
    expect << 1, 2, 34;
    assert_equal(expect, factor.evaluateError(x));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
