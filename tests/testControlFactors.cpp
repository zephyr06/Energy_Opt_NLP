#include "../sources/ControlOptimize.h"

using namespace std;
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
TEST(coeffFactor, v1)
{
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    weightSchedulability = 1;
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
    weightSchedulability = 1;
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

TEST(BuildGraph, v1)
{
    weightSchedulability = 1;
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

TEST(RTAFactor, J)
{
    noiseModelSigma = 1;
    weightSchedulability = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    auto factor1 = GenerateTaskRTAFactor(maskForElimination, tasks, 4);
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
    }
    std::vector<MatrixDynamic> Hs, HsExpect;
    Hs.reserve(10);
    HsExpect.reserve(10);
    for (uint i = 0; i < 10; i++)
    {
        MatrixDynamic m = GenerateOneMatrix(1, 1);
        Hs.push_back(m);
        HsExpect.push_back(m);
    }

    auto eActual = factor1.unwhitenedError(initialEstimateFG, Hs);
    cout << "Hs is as follows: " << endl;
    for (uint i = 0; i < 10; i++)
    {
        cout << Hs[i] << endl;
    }
    AssertEigenEqualMatrix(GenerateVectorDynamic1D(1), Hs[9]);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
