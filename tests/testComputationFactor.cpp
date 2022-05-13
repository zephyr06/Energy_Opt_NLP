#include <CppUnitLite/TestHarness.h>
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/Tools/testMy.h"

using namespace rt_num_opt;
using namespace gtsam;
using namespace std;
TEST(evaluateError, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v23.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    TaskSetNormal tasksN(tasks);
    int N = tasks.size();
    InitializeGlobalVector(tasks.size());
    int lastTaskDoNotNeedOptimize = -1;
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    gtsam::Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma((N - lastTaskDoNotNeedOptimize - 1), noiseModelSigma);
    Energy_Opt<TaskSetNormal, RTA_LL>::ComputationFactor factor(key, tasksN, -1, responseTimeInitial, model);
    VectorDynamic x = GetParameterVD<double>(tasks, "executionTime");
    executionTimeModel = 1;
    EnergyMode = 1;
    VectorDynamic expect = x;
    expect << 20 / 400.0, 10.0 / 200, 80 / 400.0;
    expect *= weightEnergy;
    // EXPECT(assert_equal(expect, factor.evaluateError(x)));
}

TEST(evaluateError, v2)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v24.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    TaskSetNormal tasksN(tasks);
    int N = tasks.size();
    InitializeGlobalVector(tasks.size());
    int lastTaskDoNotNeedOptimize = -1;
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma((N - lastTaskDoNotNeedOptimize - 1), noiseModelSigma);
    Energy_Opt<TaskSetNormal, RTA_LL>::ComputationFactor factor(key, tasksN, -1, responseTimeInitial, model);
    VectorDynamic x = GetParameterVD<double>(tasks, "executionTime");
    x(0) = 50;
    executionTimeModel = 1;
    EnergyMode = 1;
    VectorDynamic expect = x;
    weightEnergy = 1e8;
    expect << 0.008, 10.0 / 200, 80 / 400.0;
    expect *= weightEnergy;
    MaxComputationTimeRestrict = 5;
    expect(0) += punishmentInBarrier * 10;
    // EXPECT(assert_equal(expect, factor.evaluateError(x)));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
