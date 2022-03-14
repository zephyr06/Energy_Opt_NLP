#include "../sources/ControlOptimize.h"

using namespace std;
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
// TEST(coeffFactor, v1)
// {
//     // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     // TaskSet tasks;
//     // VectorDynamic coeff;
//     // std::tie(tasks, coeff) = ReadControlCase(path1);
//     weightSchedulability = 1;
//     auto model = noiseModel::Isotropic::Sigma(1, 1);
//     VectorDynamic coeff0 = GenerateVectorDynamic(1);
//     coeff0 << 3;
//     VectorDynamic x = coeff0;
//     x << 1;
//     Symbol key('a', 0);
//     CoeffFactor f0(key, coeff0, model);
//     AssertEigenEqualVector(coeff0, f0.evaluateError(x));
//     x << 0;
//     MatrixDynamic jActual = x;
//     AssertEigenEqualVector(x, f0.evaluateError(x, jActual));
//     VectorDynamic jExpect = coeff0;
//     AssertEigenEqualVector(jExpect, jActual);
// }
// TEST(RTAFactor, v1)
// {
//     noiseModelSigma = 1;
//     weightSchedulability = 1;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     std::vector<bool> maskForElimination(tasks.size(), false);
//     NonlinearFactorGraph graph;
//     AddRTAFactor(graph, maskForElimination, tasks);
//     // initial estimate
//     RTA_LL r(tasks);
//     auto rta = r.ResponseTimeOfTaskSet();
//     Values initialEstimateFG;
//     for (uint i = 0; i < tasks.size(); i++)
//     {
//         initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
//         initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
//     }
//     AssertEqualScalar(0, graph.error(initialEstimateFG));
//     initialEstimateFG.update(GenerateControlKey(0, "response"), GenerateVectorDynamic1D(1));
//     AssertEqualScalar(0.5, graph.error(initialEstimateFG));
//     initialEstimateFG.update(GenerateControlKey(4, "response"), GenerateVectorDynamic1D(120));
//     AssertEqualScalar(25, graph.error(initialEstimateFG));
//     initialEstimateFG.update(GenerateControlKey(4, "period"), GenerateVectorDynamic1D(60));
//     AssertEqualScalar(25, graph.error(initialEstimateFG));
//     initialEstimateFG.update(GenerateControlKey(3, "period"), GenerateVectorDynamic1D(60));
//     AssertEqualScalar(1458.5, graph.error(initialEstimateFG));
// }

// TEST(BuildGraph, v1)
// {
//     weightSchedulability = 1;
//     noiseModelSigma = 1;
//     weightHardConstraint = 1;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     coeff << 1, 1, 1, 1, 1,
//         1, 1, 1, 1, 1;
//     std::vector<bool> maskForElimination(tasks.size(), false);
//     NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
//     RTA_LL r(tasks);
//     auto rta = r.ResponseTimeOfTaskSet();
//     Values initialEstimateFG;
//     for (uint i = 0; i < tasks.size(); i++)
//     {
//         initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
//         initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
//     }
//     AssertEqualScalar(1026303.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
//     initialEstimateFG.update(GenerateControlKey(0, "response"), GenerateVectorDynamic1D(1));
//     initialEstimateFG.update(GenerateControlKey(0, "period"), GenerateVectorDynamic1D(200));
//     initialEstimateFG.update(GenerateControlKey(1, "response"), GenerateVectorDynamic1D(51));
//     initialEstimateFG.update(GenerateControlKey(2, "response"), GenerateVectorDynamic1D(636));
//     AssertEqualScalar(1176775.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
// }
TEST(BuildGraph, v2)
{
    weightSchedulability = 1;
    noiseModelSigma = 1;
    weightHardConstraint = 100;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);

    std::vector<bool> maskForElimination(tasks.size(), false);
    NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    Values initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    AssertEqualScalar(436274310542.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
    // initialEstimateFG.update(GenerateControlKey(1000, "response"), GenerateVectorDynamic1D(1));
    // AssertEqualScalar(25948729015644.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
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
        MatrixDynamic m = GenerateMatrixDynamic(1, 1);
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
TEST(GenerateSchedulabilityFactor, v1)
{
    weightSchedulability = 1;
    weightHardConstraint = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    periodInitial1 << 127.008,
        127.077,
        223.425,
        127.002,
        114.262;
    UpdateTaskSetPeriod(tasks, periodInitial1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[0] = 1;
    maskForElimination[3] = 1;
    auto factor1 = FactorGraphForceManifold::GenerateSchedulabilityFactor(maskForElimination, tasks, 4);
    AssertEqualScalar(127 - 114.262, factor1.evaluateError(
                                         GenerateVectorDynamic1D(127), GenerateVectorDynamic1D(114.262))(0, 0));
}

TEST(GenerateSchedulabilityFactor, v2)
{
    weightSchedulability = 1;
    weightHardConstraint = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    coeff << 1, 2, 1, 4, 1, 1, 1, 1, 1, 1;
    std::vector<bool> maskForElimination(tasks.size(), false);
    auto factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 0, coeff);
    auto e2Expect = GenerateVectorDynamic(2);
    e2Expect << 4, 0;
    auto initial = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial), __LINE__);

    initial.update(GenerateControlKey(0, "period"), GenerateVectorDynamic1D(70));
    initial.update(GenerateControlKey(3, "period"), GenerateVectorDynamic1D(110));
    factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 3, coeff);
    e2Expect << INT32_MAX, INT32_MAX - 110;
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial), __LINE__);
}

TEST(GenerateSchedulabilityFactor, v3)
{
    weightSchedulability = 1;
    weightHardConstraint = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    coeff << 1, 2, 1, 4, 1, 1, 1, 1, 1, 1;
    std::vector<bool> maskForElimination(tasks.size(), false);
    auto factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 0, coeff);
    auto e2Expect = GenerateVectorDynamic(2);
    e2Expect << 4, 0;
    auto initial = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial), __LINE__);

    initial.update(GenerateControlKey(0, "period"), GenerateVectorDynamic1D(70));

    factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 3, coeff);
    e2Expect << 117, 0;

    std::vector<MatrixDynamic> HsExpect, HsActual;
    HsExpect.push_back(GenerateMatrixDynamic(2, 1));
    HsExpect.push_back(GenerateMatrixDynamic(2, 1));
    HsExpect.push_back(GenerateMatrixDynamic(2, 1));
    HsActual = HsExpect;
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial, HsActual), __LINE__);
    AssertEigenEqualMatrix(HsExpect[0], HsActual[0], __LINE__);
    AssertEigenEqualMatrix(HsExpect[1], HsActual[1], __LINE__);
    AssertEigenEqualMatrix(HsExpect[2], HsActual[2], __LINE__);
    initial.update(GenerateControlKey(3, "period"), GenerateVectorDynamic1D(110));
    e2Expect << INT32_MAX, INT32_MAX - 110;
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial, HsActual), __LINE__);
    auto matrix1 = GenerateMatrixDynamic(2, 1);

    HsExpect[2] = matrix1;
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
