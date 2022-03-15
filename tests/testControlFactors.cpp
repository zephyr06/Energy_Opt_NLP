#include "../sources/ControlOptimize.h"
#include "../sources/BatchControlOptimize.h"
using namespace std;
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
TEST(ExtractResults, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
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
    AssertEigenEqualVector(expectT, FactorGraphForceManifold::ExtractResults(result, tasks).first);
    AssertEigenEqualVector(expectR, FactorGraphForceManifold::ExtractResults(result, tasks).second);
}
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
    RTA_LL r(tasks);
    VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
    NonlinearFactorGraph graph;
    for (int index = 0; index < int(tasks.size()); index++)
    {
        MultiKeyFactor f = GenerateTaskRTAFactor(maskForElimination, tasks, index, rtaBase);
        graph.add(f);
    }
    // initial estimate
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rtaBase(i, 0)));
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
    weightHardConstraint = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    coeff << 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1;
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
    NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
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
TEST(BuildGraph, v2)
{
    weightSchedulability = 1;
    noiseModelSigma = 1;
    weightHardConstraint = 100;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);

    std::vector<bool> maskForElimination(tasks.size() * 2, false);
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
    auto factor1 = GenerateTaskRTAFactor(maskForElimination, tasks, 4, rta);
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
    std::cout << "Hs is as follows: " << endl;
    for (uint i = 0; i < 10; i++)
    {
        std::cout << Hs[i] << endl;
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
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
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

TEST(FindEliminatedVariables, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);

    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 45, 372.719, 454.248, 128.127, 358.683;
    UpdateTaskSetPeriod(tasks, initial);
    FactorGraphInManifold::FindEliminatedVariables(tasks, maskForElimination, 1);
    AssertEqualVectorExact({1, 0, 0, 0, 0}, maskForElimination);
}

TEST(FactorGraphInManifold, inference)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 68.0034,
        321.249,
        400.807,
        131.088,
        308.676;
    UpdateTaskSetPeriod(tasks, initial);
    NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();

    MatrixDynamic jacobianCurr = sth.first;
    std::cout << "Current Jacobian matrix:" << endl;
    std::cout << jacobianCurr << endl;
    std::cout << "Current b vector: " << endl;
    std::cout << sth.second << endl;
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(12, 4);
    jacobianExpect(0, 0) = 275;
    jacobianExpect(3, 1) = 217;
    jacobianExpect(6, 2) = 489;
    jacobianExpect(9, 3) = 285;
    AssertEigenEqualMatrix(jacobianExpect, jacobianCurr);
    AssertEqualScalar(321.249 * -1 * 275, sth.second(0, 0));
    AssertEqualScalar(-50 * 9334, sth.second(1, 0));
}

TEST(FactorGraphInManifold, inference2)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 68.0034,
        321.249,
        400.807,
        131.088,
        308.676;
    UpdateTaskSetPeriod(tasks, initial);
    NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    FactorGraphInManifold::FindEliminatedVariables(tasks, maskForElimination);
    AssertEqualVectorExact({1, 0, 0, 0, 0}, maskForElimination);
}
TEST(io, IfTargetFile)
{
    string s1 = "Case0.m";
    string s2 = "Case0.txt";
    string s3 = "Case8.txt_RM_GPResult.txt";
    string s4 = "Case20.txt_RM_BFSResult.txt";
    string s5 = "";
    string s6 = ".";
    string s7 = "../";
    AssertEqualScalar(3, TargetFileType(s1));
    AssertEqualScalar(3, TargetFileType(s5));
    AssertEqualScalar(3, TargetFileType(s6));
    AssertEqualScalar(3, TargetFileType(s7));
    AssertEqualScalar(0, TargetFileType(s2));
    AssertEqualScalar(1, TargetFileType(s3));
    AssertEqualScalar(2, TargetFileType(s4));
    string path = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt_RM_BFSResult.txt";
    AssertEqualScalar(0.00995458, ReadBaselineZhao20(path).first);
    AssertEqualScalar(1.52131e+06, ReadBaselineZhao20(path).second);
    path = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case9.txt_RM_GPResult.txt";
    AssertEqualScalar(0.136000, ReadBaselineZhao20(path).first);
    AssertEqualScalar(2852692.008127, ReadBaselineZhao20(path).second);
}

TEST(jacobian, vn)
{
    noiseModelSigma = 1;
    weightSchedulability = 1e6;
    weightHardConstraint = 1e5;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
    maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 127.008,
        455.654,
        515.147,
        237.366,
        444.963;
    UpdateTaskSetPeriod(tasks, initial);
    NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    auto initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();

    MatrixDynamic jacobianCurr = sth.first;
    std::cout << "Current Jacobian matrix:" << endl;
    std::cout << jacobianCurr << endl;
    std::cout << "Current b vector: " << endl;
    std::cout << sth.second << endl;
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(18, 9);

    AssertEqualScalar(1e6, jacobianCurr(2, 1));
    AssertEqualScalar(1e6, jacobianCurr(0, 0));
}
TEST(QuotientDouble, v1)
{
    AssertEqualScalar(0.01, QuotientDouble(127.01, 127));
    AssertEqualScalar(0.01, QuotientDouble(126.99, 127));
    AssertEqualScalar(0.03, QuotientDouble(126.99 * 3, 127));
    AssertEqualScalar(126.97, QuotientDouble(126.99 * 3, 127 * 2));
    AssertEqualScalar(0, QuotientDouble(10, 2));
    AssertEqualScalar(2, QuotientDouble(2, 10));
    AssertEqualScalar(4.9, QuotientDouble(4.9, 10));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
