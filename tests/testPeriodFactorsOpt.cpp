#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/ReadControlCases.h"
#include "../sources/CoeffFactor.h"
#include "../sources/RTAFactor.h"
#include "../sources/InequalifyFactor.h"
using namespace std;
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
TEST(coeffFactor, v1)
{
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    auto model = noiseModel::Isotropic::Sigma(1, 1);
    VectorDynamic coeff0 = GenerateVectorDynamic(1);
    coeff0 << 3;
    VectorDynamic x = coeff0;
    x << 1;
    Symbol key('a', 0);
    CoeffFactor f0(key, coeff0, model);
    AssertEigenEqualVector(coeff0, f0.evaluateError(x));
    x << 0;
    MatrixDynamic jActual = x;
    AssertEigenEqualVector(x, f0.evaluateError(x, jActual));
    VectorDynamic jExpect = coeff0;
    AssertEigenEqualVector(jExpect, jActual);
}
TEST(RTAFactor, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    NonlinearFactorGraph graph;
    AddRTAFactor(graph, maskForElimination, tasks);
    // initial estimate
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
    }
    AssertEqualScalar(0, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateControlKey(0, "response"), GenerateVectorDynamic1D(1));
    AssertEqualScalar(0.5, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateControlKey(4, "response"), GenerateVectorDynamic1D(120));
    AssertEqualScalar(25, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateControlKey(4, "period"), GenerateVectorDynamic1D(60));
    AssertEqualScalar(25, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateControlKey(3, "period"), GenerateVectorDynamic1D(60));
    AssertEqualScalar(1458.5, graph.error(initialEstimateFG));
}

NonlinearFactorGraph BuildControlGraph(std::vector<bool> maskForElimination, TaskSet tasks, VectorDynamic &coeff)
{
    NonlinearFactorGraph graph;
    double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
    auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    for (uint i = 0; i < tasks.size(); i++)
    {
        // add CoeffFactor
        graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(coeff(2 * i, 0)), model1);
        graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(coeff(2 * i + 1, 0)), model1);

        // add RTAFactor
        MultiKeyFactor f = GenerateTaskRTAFactor(maskForElimination, tasks, i);
        graph.add(f);

        // add period min/max limits
        graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "period"), 0, model1);
        graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "period"), periodMax, model1);

        // schedulability
        graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "response"), min(tasks[i].period, tasks[i].deadline), model1);
    }
    return graph;
}

TEST(BuildGraph, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    coeff << 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1;
    std::vector<bool> maskForElimination(tasks.size(), false);
    NonlinearFactorGraph graph = BuildControlGraph(maskForElimination, tasks, coeff);
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
    }
    AssertEqualScalar(1026303.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
    initialEstimateFG.update(GenerateControlKey(0, "response"), GenerateVectorDynamic1D(1));
    initialEstimateFG.update(GenerateControlKey(0, "period"), GenerateVectorDynamic1D(200));
    initialEstimateFG.update(GenerateControlKey(1, "response"), GenerateVectorDynamic1D(51));
    initialEstimateFG.update(GenerateControlKey(2, "response"), GenerateVectorDynamic1D(636));
    AssertEqualScalar(1176775.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
