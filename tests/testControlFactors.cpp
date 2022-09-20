#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/BatchControlOptimize.h"
using namespace std;
using namespace std::chrono;

using namespace rt_num_opt;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
using namespace ControlOptimize;
using namespace std;

using namespace gtsam;
// TEST(ExtractResults, v1)
// {
//     noiseModelSigma = 1;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     std::vector<bool> maskForElimination(tasks.size() * 2, false);
//     maskForElimination[1] = true;
//     Values result;
//     result.insert(GenerateKey(0, "period"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(2, "period"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(3, "period"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(4, "period"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(0, "response"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(1, "response"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(2, "response"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(3, "response"), GenerateVectorDynamic1D(1));
//     result.insert(GenerateKey(4, "response"), GenerateVectorDynamic1D(1));
//     VectorDynamic expectT = GenerateVectorDynamic(5);
//     expectT = expectT.array() + 1;
//     VectorDynamic expectR = expectT;
//     expectT(1, 0) = tasks[1].period;
//     AssertEigenEqualVector(expectT, FactorGraphForceManifold::ExtractResults(result, tasks).first);
//     AssertEigenEqualVector(expectR, FactorGraphForceManifold::ExtractResults(result, tasks).second);
// }

TEST(coeffFactor, v1)
{
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    jacobianScale = 1;
    weightSchedulability = 1;
    whether_ls = 1;
    if (optimizerType >= 5)
        optimizerType = 2;
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
    gtsam::NonlinearFactorGraph graph;
    for (int index = 0; index < int(tasks.size()); index++)
    {
        MultiKeyFactor f = GenerateTaskRTAFactor(maskForElimination, tasks, index, rtaBase);
        graph.add(f);
    }
    // initial estimate
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateKey(i, "response"), GenerateVectorDynamic1D(rtaBase(i, 0)));
    }
    AssertEqualScalar(0, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateKey(0, "response"), GenerateVectorDynamic1D(1));
    AssertEqualScalar(0.5, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateKey(4, "response"), GenerateVectorDynamic1D(120));
    AssertEqualScalar(25, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateKey(4, "period"), GenerateVectorDynamic1D(60));
    AssertEqualScalar(25, graph.error(initialEstimateFG));
    initialEstimateFG.update(GenerateKey(3, "period"), GenerateVectorDynamic1D(60));
    AssertEqualScalar(1458.5, graph.error(initialEstimateFG));
}

TEST(BuildGraph, FactorGraphForceManifold)
{
    // weightSchedulability = 1;
    // noiseModelSigma = 1;
    // weightHardConstraint = 1;
    // exactJacobian = 0;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    // coeff << 1, 1, 1, 1, 1,
    //     1, 1, 1, 1, 1;
    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // gtsam::NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // RTA_LL r(tasks);
    // auto rta = r.ResponseTimeOfTaskSet();
    // gtsam::Values initialEstimateFG;
    // for (uint i = 0; i < tasks.size(); i++)
    // {
    //     initialEstimateFG.insert(GenerateKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
    //     initialEstimateFG.insert(GenerateKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
    // }
    // AssertEqualScalar(1026303.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
    // initialEstimateFG.update(GenerateKey(0, "response"), GenerateVectorDynamic1D(1));
    // initialEstimateFG.update(GenerateKey(0, "period"), GenerateVectorDynamic1D(200));
    // initialEstimateFG.update(GenerateKey(1, "response"), GenerateVectorDynamic1D(51));
    // initialEstimateFG.update(GenerateKey(2, "response"), GenerateVectorDynamic1D(636));
    // AssertEqualScalar(1176775.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
}
TEST(BuildGraph, v2)
{
    // weightSchedulability = 1;
    // noiseModelSigma = 1;
    // weightHardConstraint = 100;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);

    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // gtsam::NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // RTA_LL r(tasks);
    // auto rta = r.ResponseTimeOfTaskSet();
    // gtsam::Values initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    // AssertEqualScalar(436274310542.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
    // // initialEstimateFG.update(GenerateKey(1000, "response"), GenerateVectorDynamic1D(1));
    // // AssertEqualScalar(25948729015644.5, graph.error(initialEstimateFG), 1e-6, __LINE__);
}

TEST(RTAFactor, J)
{
    noiseModelSigma = 1;
    weightSchedulability = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    // eliminationRecordGlobal.Initialize(tasks.size());
    std::vector<bool> maskForElimination(tasks.size() * 2, false);
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    auto factor1 = GenerateTaskRTAFactor(maskForElimination, tasks, 4, rta);
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        initialEstimateFG.insert(GenerateKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
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
    // std::cout << "Hs is as follows: " << endl;
    // for (uint i = 0; i < 10; i++)
    // {
    //     std::cout << Hs[i] << endl;
    // }
    AssertEigenEqualMatrix(GenerateVectorDynamic1D(1), Hs[9]);
    // assert_equal(GenerateVectorDynamic1D(1), Hs[9]);
}
TEST(GenerateSchedulabilityFactor, v1)
{
    // weightSchedulability = 1;
    // weightHardConstraint = 1;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    // // eliminationRecordGlobal.Initialize(tasks.size());
    // VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    // periodInitial1 << 127.008,
    //     127.077,
    //     223.425,
    //     127.002,
    //     114.262;
    // UpdateTaskSetPeriod(tasks, periodInitial1);
    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // maskForElimination[0] = 1;
    // maskForElimination[3] = 1;
    // auto factor1 = FactorGraphForceManifold::GenerateSchedulabilityFactor(maskForElimination, tasks, 4);
    // AssertEqualScalar(127 - 114.262, factor1.evaluateError(
    //                                      GenerateVectorDynamic1D(127), GenerateVectorDynamic1D(114.262))(0, 0));
}

TEST(GenerateSchedulabilityFactor, v2)
{
    weightSchedulability = 1;
    weightHardConstraint = 1;
    whether_ls = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    // eliminationRecordGlobal.Initialize(tasks.size());
    coeff << 1, 2, 1, 4, 1, 1, 1, 1, 1, 1;
    std::vector<bool> maskForElimination(tasks.size(), false);
    VectorDynamic rtaBase = RTAVector(tasks);
    auto factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 0, coeff, rtaBase);
    auto e2Expect = GenerateVectorDynamic(2);
    e2Expect << 4, 0;
    auto initial = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial), __LINE__);

    initial.update(GenerateKey(0, "period"), GenerateVectorDynamic1D(70));
    initial.update(GenerateKey(3, "period"), GenerateVectorDynamic1D(110));
    factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 3, coeff, rtaBase);
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
    VectorDynamic rtaBase = RTAVector(tasks);
    std::vector<bool> maskForElimination(tasks.size(), false);
    auto factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 0, coeff, rtaBase);
    auto e2Expect = GenerateVectorDynamic(2);
    e2Expect << 4, 0;
    auto initial = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    AssertEigenEqualVector(e2Expect, factor.unwhitenedError(initial), __LINE__);

    initial.update(GenerateKey(0, "period"), GenerateVectorDynamic1D(70));

    factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, 3, coeff, rtaBase);
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
    initial.update(GenerateKey(3, "period"), GenerateVectorDynamic1D(110));
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
    exactJacobian = 0;
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
    // gtsam::NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // using namespace FactorGraphInManifold;
    gtsam::NonlinearFactorGraph graph;
    double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
    auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
    VectorDynamic rtaBase = RTAVector(tasks);
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (!maskForElimination[i])
        {
            // add CoeffFactor
            graph.emplace_shared<CoeffFactor>(GenerateKey(i, "period"),
                                              GenerateVectorDynamic1D(coeff(2 * i, 0)), modelNormal);
            // add period min/max limits
            graph.emplace_shared<LargerThanFactor1D>(GenerateKey(i, "period"), tasks[i].executionTime, modelPunishmentSoft1);
            graph.emplace_shared<SmallerThanFactor1D>(GenerateKey(i, "period"), periodMax, modelPunishmentSoft1);
        }
        if (FactorGraphInManifold::HasDependency(i, maskForElimination))
        {
            auto factor = FactorGraphInManifold::GenerateRTARelatedFactor(maskForElimination, tasks, i, coeff, rtaBase);
            graph.add(factor);
        }
    }

    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();
    MatrixDynamic jacobianCurr = sth.first;
    // std::cout << "Current Jacobian matrix:" << endl;
    // std::cout << jacobianCurr << endl;
    // std::cout << "Current b vector: " << endl;
    // std::cout << sth.second << endl;
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
    disturb_max = 1;
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
    gtsam::NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
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
}

TEST(jacobian, vn)
{
    // noiseModelSigma = 1;
    // weightSchedulability = 1e6;
    // weightHardConstraint = 1e5;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // maskForElimination[0] = 1;
    // VectorDynamic initial = GenerateVectorDynamic(5);
    // initial << 127.008,
    //     455.654,
    //     515.147,
    //     237.366,
    //     444.963;
    // UpdateTaskSetPeriod(tasks, initial);
    // gtsam::NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // auto initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    // auto sth = graph.linearize(initialEstimateFG)->jacobian();

    // MatrixDynamic jacobianCurr = sth.first;
    // // std::cout << "Current Jacobian matrix:" << endl;
    // // std::cout << jacobianCurr << endl;
    // // std::cout << "Current b vector: " << endl;
    // // std::cout << sth.second << endl;
    // MatrixDynamic jacobianExpect = GenerateMatrixDynamic(18, 9);

    // AssertEqualScalar(1e6, jacobianCurr(2, 1));
    // AssertEqualScalar(1e6, jacobianCurr(0, 0));
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
TEST(FindEliminatedVariables, ForceManifold)
{
    // noiseModelSigma = 1;
    // weightSchedulability = 1e6;
    // weightHardConstraint = 1e5;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // VectorDynamic initial = GenerateVectorDynamic(5);
    // initial << 127.008,
    //     455.654,
    //     515.147,
    //     237.366,
    //     444.963;
    // UpdateTaskSetPeriod(tasks, initial);
    // // NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // // auto initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    // // rta: 2, 50, 68, 115, 127
    // FactorGraphForceManifold::FindEliminatedVariables(tasks, maskForElimination);
    // AssertBool(true, maskForElimination[9]); // r_4
    // AssertBool(true, maskForElimination[0]); // t_0
}
TEST(eliminate, ForceManifold)
{
    // noiseModelSigma = 1;
    // weightSchedulability = 1e6;
    // weightHardConstraint = 1e5;
    // std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    // TaskSet tasks;
    // VectorDynamic coeff;
    // std::tie(tasks, coeff) = ReadControlCase(path1);
    // std::vector<bool> maskForElimination(tasks.size() * 2, false);
    // VectorDynamic initial = GenerateVectorDynamic(5);
    // initial << 127.008,
    //     455.654,
    //     515.147,
    //     237.366,
    //     444.963;
    // UpdateTaskSetPeriod(tasks, initial);
    // maskForElimination[0] = true;
    // maskForElimination[9] = true;

    // gtsam::NonlinearFactorGraph graph = FactorGraphForceManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // auto initialEstimateFG = FactorGraphForceManifold::GenerateInitialFG(tasks, maskForElimination);
    // auto sth = graph.linearize(initialEstimateFG)->jacobian();

    // MatrixDynamic jacobianCurr = sth.first;
    // // std::cout << "Current Jacobian matrix:" << endl;
    // // std::cout << jacobianCurr << endl;
    // // std::cout << "Current b vector: " << endl;
    // // std::cout << sth.second << endl;
    // AssertEqualScalar(1e6, jacobianCurr(0, 0));
    // AssertEqualScalar(1e6, jacobianCurr(2, 1));
}
TEST(HasDependency, v1)
{
    noiseModelSigma = 1;
    weightSchedulability = 1e6;
    weightHardConstraint = 1e5;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination = {1, 0, 1, 1, 1};
    AssertEqualScalar(false, FactorGraphInManifold::HasDependency(0, maskForElimination), 1e-6, __LINE__);
    AssertEqualScalar(true, FactorGraphInManifold::HasDependency(1, maskForElimination), 1e-6, __LINE__);
    AssertEqualScalar(true, FactorGraphInManifold::HasDependency(2, maskForElimination), 1e-6, __LINE__);
    AssertEqualScalar(true, FactorGraphInManifold::HasDependency(3, maskForElimination), 1e-6, __LINE__);
    AssertEqualScalar(true, FactorGraphInManifold::HasDependency(4, maskForElimination), 1e-6, __LINE__);
}
TEST(ExtractCaseID, v1)
{
    EXPECT_LONGS_EQUAL(0, ExtractCaseID("Case0.txt_RM_GPResult.txt"));
    EXPECT_LONGS_EQUAL(6, ExtractCaseID("Case6.txt_RM_GPResult.txt"));
    EXPECT_LONGS_EQUAL(215, ExtractCaseID("Case215.txt_RM_GPResult.txt"));
    EXPECT_LONGS_EQUAL(31, ExtractCaseID("Case31.txt_RM_GPResult.txt"));
}
// TEST(ReadBaselineZhao20, unschedulable)
// {
//     string directory = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N20/";
//     auto res1 = ReadBaselineZhao20(directory, "Case0.txt_RM_GPResult.txt");
//     EXPECT_LONGS_EQUAL(1402.473, res1.first);
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N20/Case0.txt";
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     EXPECT_LONGS_EQUAL(54382956, res1.second);

//     res1 = ReadBaselineZhao20("/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/", "Case0.txt_RM_GPResult.txt");
//     EXPECT_LONGS_EQUAL(0.543000, res1.first);
//     EXPECT_LONGS_EQUAL(1521314.003946, res1.second);

//     res1 = ReadBaselineZhao20("/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/", "Case0.txt_RM_BFSResult.txt");
//     EXPECT_LONGS_EQUAL(0.00995458, res1.first);
//     EXPECT_LONGS_EQUAL(1.52131e+06, res1.second);
// }

TEST(ReadBaselineZhao20, unschedulable_v2)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N20/Case3.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    RTA_LL r(tasks);
    auto res = r.CheckSchedulability(false);
    EXPECT(res);
    VectorDynamic periodCurr = GenerateVectorDynamic(20);
    // periodCurr << 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 387, 774, 774, 387, 774, 774, 774;
    periodCurr = periodCurr.array() + 1032;
    UpdateTaskSetPeriod(tasks, periodCurr);
    tasks = Reorder(tasks, "RM");
    RTA_LL r2(tasks);
    EXPECT(r2.CheckSchedulability(false));
}
// TEST(ReadBaselineZhao20, unschedulable_v3)
// {
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N20/Case684.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     RTA_LL r(tasks);
//     auto res = r.CheckSchedulability(1);
//     EXPECT(res);
//     VectorDynamic periodCurr = GenerateVectorDynamic(20);
//     // periodCurr << 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 774, 387, 774, 774, 387, 774, 774, 774;
//     periodCurr = periodCurr.array() + 1133;
//     UpdateTaskSetPeriod(tasks, periodCurr);
//     tasks = Reorder(tasks, "RM");
//     RTA_LL r2(tasks);
//     EXPECT(r2.CheckSchedulability(1));
// }
TEST(Reorder, v1)
{
    enableReorder = 1;
    TaskSet tasks;
    VectorDynamic coeff;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    std::tie(tasks, coeff) = ReadControlCase(path1);
    tasks[0].period = 5;
    tasks[1].period = 10;
    tasks[2].period = 7;
    tasks[3].period = 7;
    tasks[4].period = 7;
    Reorder(tasks, coeff);
    EXPECT_LONGS_EQUAL(0, tasks[0].id);

    EXPECT_LONGS_EQUAL(2, tasks[1].id);

    EXPECT_LONGS_EQUAL(3, tasks[2].id);

    EXPECT_LONGS_EQUAL(4, tasks[3].id);

    EXPECT_LONGS_EQUAL(1, tasks[4].id);
    VectorDynamic coeffExpect = coeff;
    coeffExpect << 645, 7143, 217, 5031, 489, 3778, 285, 380, 275, 9334;
    assert_equal(coeffExpect, coeff);
}
TEST(Reorder, v2)
{
    enableReorder = 1;
    TaskSet tasks;
    VectorDynamic coeff;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N20/Case0.txt";
    std::tie(tasks, coeff) = ReadControlCase(path1);
    tasks[0].period = 5;
    tasks[1].period = 10;
    tasks[2].period = 7;
    tasks[3].period = 7;
    tasks[4].period = 7;
    Reorder(tasks, coeff);
    EXPECT_LONGS_EQUAL(0, tasks[0].id);
    EXPECT_LONGS_EQUAL(2, tasks[1].id);
    EXPECT_LONGS_EQUAL(3, tasks[2].id);
    EXPECT_LONGS_EQUAL(4, tasks[3].id);
    EXPECT_LONGS_EQUAL(1, tasks[4].id);
}
class TestBigJacobian1 : public NoiseModelFactor1<VectorDynamic>
{
public:
    NormalErrorFunction1D f;
    int dimension;

    TestBigJacobian1(Key key,
                     SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key)
    {
        dimension = 1;
    }

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic b = GenerateVectorDynamic(6);
        b << 1, 2, 3, 4, 5, 6;
        MatrixDynamic jj = GenerateMatrixDynamic(6, 5);
        jj << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, -1e6, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

        VectorDynamic err = jj * x - b;
        if (H)
        {
            *H = jj;
        }
        return err;
    }
};

TEST(assumption, bigJacobian)
{
    int m = 6;
    int n = 5;
    gtsam::NonlinearFactorGraph graph;
    auto key = Symbol('a', 0);
    auto model = noiseModel::Isotropic::Sigma(m, noiseModelSigma);
    graph.emplace_shared<TestBigJacobian1>(key, model);

    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    gtsam::Values initialEstimateFG;
    initialEstimateFG.insert(key, GenerateVectorDynamic(n));

    gtsam::Values result;
    if (optimizerType == 1)
    {
        gtsam::DoglegParams params;
        // if (debugMode == 1)
        //     params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        gtsam::DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        gtsam::LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        params.setVerbosityLM(verbosityLM);
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    cout << result.at<VectorDynamic>(key);
    EXPECT_LONGS_EQUAL(-3e-6, result.at<VectorDynamic>(key)(2));
}

TEST(ControlObjFactor, jacobian)
{
    whether_ls = 0;
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case857.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    // maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 1000, 1000, 1000, 1000, 1000;
    UpdateTaskSetPeriod(tasks, initial);
    // gtsam::NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    // using namespace FactorGraphInManifold;
    gtsam::NonlinearFactorGraph graph;
    double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
    auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
    VectorDynamic rtaBase = RTAVector(tasks);
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (FactorGraphInManifold::HasDependency(i, maskForElimination))
        {
            auto factor = FactorGraphInManifold::GenerateControlObjFactor(maskForElimination, tasks, i, coeff, rtaBase);
            graph.add(factor);
        }
    }

    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();
    MatrixDynamic jacobianCurr = sth.first;
    std::cout << "The following should be a diagonal matrix" << std::endl;
    std::cout << "Current Jacobian matrix:" << endl;
    std::cout << jacobianCurr << endl;
    std::cout << "Current b vector: " << endl;
    std::cout << sth.second << endl;
    EXPECT_DOUBLES_EQUAL(0, jacobianCurr(2, 0), 1e-5);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
