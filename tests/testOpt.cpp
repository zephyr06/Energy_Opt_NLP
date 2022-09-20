#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/EnergyOptimization/FactorGraphEnergyLL.h"
#include "sources/Utils/FactorGraphUtils.h"
using namespace std::chrono;
using namespace rt_num_opt;
using namespace std;

using namespace gtsam;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;

// There are two types of tests, strict deadline test, or period & 2xExecution test

TEST(FindTaskDoNotNeedOptimize, A1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    TaskSetNormal taskSetNormal(tasks);
    InitializeGlobalVector(tasks.size());
    int N = tasks.size();
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;

    int indexExpect = 1;
    int indexActual = Opt_LL::FindTaskDoNotNeedOptimize(taskSetNormal, 0, responseTimeInitial, eliminateTol);
    CHECK_EQUAL(indexExpect, indexActual);
}

TEST(FindTaskDoNotNeedOptimize, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    optimizerType = 1;
    EnergyMode = 1;
    eliminateTol = 0.1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    InitializeGlobalVector(taskSet1.size());
    VectorDynamic initialExecution = GetParameterVD<int>(taskSet1, "executionTime");
    double eliminateTol_t = eliminateTol;
    eliminateTol = 3.1;
    int index = Opt_LL::FindTaskDoNotNeedOptimize(taskSet1, 0, initialExecution, eliminateTol);
    AssertEqualScalar(0, index, 0.00001, __LINE__);
    eliminateTol = 198.1;
    index = Opt_LL::FindTaskDoNotNeedOptimize(taskSet1, 0, initialExecution, eliminateTol);
    AssertEqualScalar(2, index, 1e-6, __LINE__);
    eliminateTol = eliminateTol_t;
}
TEST(FindTaskDoNotNeedOptimize, a2)
{
    runMode = "normal";
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    optimizerType = 1;
    EnergyMode = 2;
    executionTimeModel = 2;
    eliminateTol = 0.1;
    exactJacobian = 0;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v8.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    InitializeGlobalVector(taskSet1.size());
    VectorDynamic initialExecution = GetParameterVD<int>(taskSet1, "executionTime");
    initialExecution << 10.7515, 84.5303, 1232.2664, 96.768, 34.85, 243.83, 600.18, 305.87, 25.12, 37.92;
    UpdateTaskSetExecutionTime(taskSet1, initialExecution);
    int index = Opt_LL::FindTaskDoNotNeedOptimize(taskSet1, -1, initialExecution, 0.1);
    AssertEqualScalar(5, index, 1e-6, __LINE__);
}
TEST(NumericalDerivativeDynamic, A1)
{
    // NumericalDerivativeDynamic
    // f: R2 -> R4
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    boost::function<gtsam::Matrix(const VectorDynamic &)> f =
        [this](const VectorDynamic &input)
    {
        int variablesNumber = 2;
        VectorDynamic err;
        err.resize(variablesNumber * 2, 1);
        for (int i = 0; i < variablesNumber; i++)
        {
            err(i, 0) = pow(input(i), 2);
            err(i + variablesNumber, 0) = pow(input(i), 2);
        }
        return err;
    };

    VectorDynamic executionTimeVector;
    executionTimeVector.resize(2, 1);
    executionTimeVector << 4, 5;
    MatrixDynamic H_Actual = NumericalDerivativeDynamic(f, executionTimeVector, deltaOptimizer, 4);
    MatrixDynamic H_Expect;
    H_Expect.resize(4, 2);
    H_Expect << 8, 0, 0, 10, 8, 0, 0, 10;
    EXPECT(assert_equal(H_Expect, H_Actual, 1e-3));
}

TEST(UpdateTaskSetExecutionTime, A1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    InitializeGlobalVector(taskSet1.size());
    VectorDynamic executionTimeVec;
    executionTimeVec.resize(1, 1);
    executionTimeVec << 80;
    UpdateTaskSetExecutionTime(taskSet1, executionTimeVec, 1);
    if (not(80 == taskSet1[2].executionTime))
        throw;
}

TEST(checkConvergenceInterior, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    optimizerType = 1;
    EnergyMode = 1;
    double oldY = 1;
    double newY = 1.01;
    VectorDynamic oldX;
    oldX.resize(2, 1);
    oldX << 4, 5;
    VectorDynamic newX = oldX;
    newX(0, 0) = 4.5;
    if (not checkConvergenceInterior(oldY, oldX, newY, newX, 1e-1, 1e-1))
    {
        cout << "Wrong in checkConvergenceInterior\n";
    }
    newX(0, 0) = 4.5 + 1e-6;
    if (not checkConvergenceInterior(oldY, oldX, newY, newX, 1e-3, 1e-1))
    {
        cout << "Wrong in checkConvergenceInterior\n";
    }
}
// ******************** performance tests ****************************
TEST(unitOptimization, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    TaskSetNormal taskSetNormal(taskSet1);
    int N = taskSet1.size();
    InitializeGlobalVector(taskSet1.size());
    optimizerType = 1;
    EnergyMode = 1;
    executionTimeModel = 1;
    int lastTaskDoNotNeedOptimize = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize((N - lastTaskDoNotNeedOptimize - 1), 1);
    initialEstimate << 62;

    RTA_LL r(taskSet1);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    vectorGlobalOpt.resize(N, 1);
    VectorDynamic res1 = Opt_LL::UnitOptimization(taskSetNormal, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);

    // 204 corresponds to RT of 319, which should be the best we can get because of the clamp function
    if (not(abs(204 - res1(0, 0)) < 5))
        CoutWarning("One test case failed in performance!");
}

TEST(OptimizeTaskSet, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    optimizerType = 1;
    EnergyMode = 1;
    executionTimeModel = 1;
    elimIte = 100;
    runMode = "normal";
    exactJacobian = 0;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    TaskSetNormal taskSetNormal(taskSet1);
    InitializeGlobalVector(taskSet1.size());

    double res = Opt_LL::OptimizeTaskSet(taskSetNormal);
    if (debugMode == 1)
        cout << "The energy saving ratio is " << res << endl;
    if (not assert_equal<double>(0.713, res, 0.01))
        CoutError("One test case failed in performance!");
}

TEST(OptimizeTaskSetOneIte, a2)
{
    enableMaxComputationTimeRestrict = 1;
    MaxComputationTimeRestrict = 2;
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";
    if (debugMode == 1)
        cout << endl
             << path << endl
             << endl;
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    InitializeGlobalVector(taskSet1.size());
    weightEnergy = 1e3;
    optimizerType = 1;
    EnergyMode = 1;
    elimIte = 100;
    runMode = "normal";
    exactJacobian = 0;
    enableMaxComputationTimeRestrict = 0;
    executionTimeModel = 1;
    TaskSetNormal taskSetNormal(taskSet1);
    double res = Opt_LL::OptimizeTaskSet(taskSetNormal);
    AssertEqualScalar(0.28, res, 0.1, __LINE__);
    if (not assert_equal<double>(0.28, res, 0.1))
        CoutError("One test case failed in performance!");
    if (debugMode == 1)
        cout << "The energy saving ratio in OptimizeTaskSet-OptimizeTaskSetOneIte is " << res << endl;
}

TEST(ClampComputationTime, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    InitializeGlobalVector(taskSet1.size());
    int N = taskSet1.size();

    int lastTaskDoNotNeedOptimize = 1;
    eliminateTol = 10;
    optimizerType = 1;
    EnergyMode = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize((N - lastTaskDoNotNeedOptimize - 1), 1);
    initialEstimate << 62;
    RTA_LL r(taskSet1);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    vectorGlobalOpt.resize(N, 1);
    TaskSetNormal taskSetNormal(taskSet1);
    VectorDynamic res1 = Opt_LL::UnitOptimization(taskSetNormal, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);

    // 204 corresponds to RT of 319, which should be the best we can get because of the clamp function
    // if (not(abs(205 - res1(0, 0)) < 1))
    //     CoutWarning("One test case failed in performance!");
    AssertEqualScalar(205, res1(0, 0), 1.1, __LINE__);
    eliminateTol = 1;
}
TEST(ClampComputationTime, v3)
{
    enableMaxComputationTimeRestrict = 1;
    MaxComputationTimeRestrict = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v23.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    InitializeGlobalVector(tasks.size());
    int N = tasks.size();

    int lastTaskDoNotNeedOptimize = -1;
    eliminateTol = 0.1;
    optimizerType = 1;
    EnergyMode = 1;
    MaxComputationTimeRestrict = 2;
    roundTypeInClamp = "fine";

    VectorDynamic initialEstimate;
    initialEstimate.resize(N, 1);
    initialEstimate << 15.2, 13.1, 12.1, 16.2, 19.5;
    UpdateTaskSetExecutionTime(tasks, initialEstimate);
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();

    TaskSetNormal taskSetNormal(tasks);
    Opt_LL::ClampComputationTime(taskSetNormal, lastTaskDoNotNeedOptimize, responseTimeInitial, "fine");
    VectorDynamic expect1 = initialEstimate;
    expect1 << 15, 13, 17, 24, 19;
    EXPECT(assert_equal(expect1, GetParameterVD<double>(taskSetNormal.tasks_, "executionTime")));
}
TEST(ClampComputationTime, v2)
{
    enableMaxComputationTimeRestrict = 1;
    MaxComputationTimeRestrict = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v22.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    TaskSetNormal taskSetNormal(tasks);
    InitializeGlobalVector(tasks.size());
    int N = tasks.size();

    int lastTaskDoNotNeedOptimize = -1;
    eliminateTol = 0.1;
    optimizerType = 1;
    EnergyMode = 1;
    roundTypeInClamp = "fine";
    MaxComputationTimeRestrict = 2;

    VectorDynamic initialEstimate;
    initialEstimate.resize(N, 1);
    initialEstimate << 15.2, 13.1, 12.1, 16.2, 19.5;
    UpdateTaskSetExecutionTime(tasks, initialEstimate);
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    Opt_LL::ClampComputationTime(taskSetNormal, lastTaskDoNotNeedOptimize, responseTimeInitial, "fine");
    VectorDynamic expect1 = initialEstimate;
    expect1 << 20, 22, 24, 26, 28;
    EXPECT(assert_equal(expect1, GetParameterVD<double>(taskSetNormal.tasks_, "executionTime")));
}
TEST(UnitOptimizationIPM, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v21.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    TaskSetNormal taskSetNormal(tasks);
    VectorDynamic initialExecution = GetParameterVD<int>(tasks, "executionTime");
    eliminateTol = 1;
    optimizerType = 1;
    EnergyMode = 1;
    enableMaxComputationTimeRestrict = 0;
    TASK_NUMBER = tasks.size();
    InitializeGlobalVector(tasks.size());
    // enableIPM = 1;
    vectorGlobalOpt.resize(3, 1);
    VectorDynamic initial;
    initial.resize(1, 1);
    initial << initialExecution(2, 0);
    VectorDynamic res = Opt_LL::UnitOptimization(taskSetNormal, 1, initial, initialExecution);
    // cout << "In unit test UnitOptimizationIPM, the res is " << res << endl;
    AssertEqualScalar(230, res(0, 0), 1.1, __LINE__);
}
// TEST(ReadYecheng_result, v1)
// {
//     string pathInPeriodicDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/periodic-set-0-syntheticJobs.csv";
//     int N = 20;
//     string yechengRepoPath = "/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep/N" + to_string(N) + "/";

//     int index = 0;
//     string targetFilePathGP = yechengRepoPath + "Case" + to_string(index) + ".txt" + "_RM_GPResult.txt";
//     string targetFilePathBF = yechengRepoPath + "Case" + to_string(index) + ".txt" + "_RM_BFSResult.txt";
//     if (debugMode == 1)
//         cout << "targetFilePathBF " << targetFilePathBF << endl;
//     string fileName;
//     if (baselineLLCompare == 1)
//         fileName = targetFilePathBF;
//     else if (baselineLLCompare == 2)
//         fileName = targetFilePathGP;
//     else
//     {
//         CoutError("Unrecognized baselineLLCompare! Current value is " + to_string(baselineLLCompare));
//     }

//     ifstream cResultFile(fileName.data());
//     try
//     {
//         assert(cResultFile.is_open());
//     }
//     catch (...)
//     {
//         cout << "Error in reading "
//              << batchOptimizeFolder
//              << "'s result files" << fileName << endl;
//     }

//     double runTime = 0, obj = 0;
//     cResultFile >> runTime >> obj;
//     double nd = 0;
//     cResultFile >> nd;
//     int n = round(nd);
//     vector<int> values(n, 0);
//     for (int i = 0; i < n; i++)
//     {
//         double val = 0;
//         cResultFile >> val;
//         values[i] = abs(round(val));
//     }

//     // check schedulability
//     auto taskSet1 = ReadTaskSet(pathInPeriodicDataset, "orig");
//     // TaskSet tasksInit = taskSet1;
//     UpdateTaskSetExecutionTime(taskSet1, Vector2Eigen(values));
//     taskSet1 = Reorder(taskSet1, "RM");
//     RTA_LL r(taskSet1);
//     // EXPECT(r.CheckSchedulability(
//     //     debugMode == 1));
// }
TEST(EnergyFactor, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto tasks = ReadTaskSet(path, "RM");
    executionTimeModel = 1;
    EnergyMode = 1;
    whether_ls = 1;
    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17, 12, 253;
    UpdateTaskSetExecutionTime(tasks, comp);

    VectorDynamic energy1;
    weightEnergy = 1e8;
    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    EnergyFactor f0(GenerateKey(0, "executionTime"), tasks[0], 0, model);
    EnergyFactor f1(GenerateKey(1, "executionTime"), tasks[1], 1, model);
    MatrixDynamic h1 = GenerateMatrixDynamic(1, 1);
    MatrixDynamic h1Expect = GenerateVectorDynamic1D(-2 / tasks[0].period * weightEnergy);
    MatrixDynamic h2 = GenerateMatrixDynamic(1, 1);
    MatrixDynamic h2Expect = GenerateVectorDynamic1D(-2 / tasks[1].period * weightEnergy);
    AssertEqualScalar(3777777.7778, f0.evaluateError(GenerateVectorDynamic1D(tasks[0].executionTime), h1)(0, 0));
    AssertEqualScalar(2448979.5918, f1.evaluateError(GenerateVectorDynamic1D(tasks[1].executionTime), h2)(0, 0));
    assert_equal(h1Expect, h1, 1e-3);
    assert_equal(h2Expect, h2, 1e-3);
}

EliminationRecord eliminationRecordGlobal;
TEST(RTARelatedFactor, v1)
{
    exactJacobian = 1;

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";
    auto tasks = ReadTaskSet(path, "RM");
    std::vector<bool> maskForElimination(tasks.size(), false);
    eliminationRecordGlobal.Initialize(tasks.size());
    VectorDynamic rtaBase = RTAVector(tasks);

    EliminationRecord eliminationRecord;
    eliminationRecord.Initialize(tasks.size());

    TaskSetNormal tasksN(tasks);
    auto f1 = GenerateRTARelatedFactor<TaskSetNormal, RTA_LL>(tasksN, eliminationRecord);
    auto x = EnergyOptUtils::GenerateInitialFG(tasksN, eliminationRecord);
    std::vector<MatrixDynamic> Hs, HsExpect;
    Hs.reserve(5);
    HsExpect.reserve(5);
    for (uint i = 0; i < 5; i++)
    {
        MatrixDynamic m = GenerateMatrixDynamic(1, 1);
        Hs.push_back(m);
        HsExpect.push_back(m);
    }
    assert_equal(GenerateVectorDynamic1D(0), f1.unwhitenedError(x, Hs), 1e-3);
    for (int i = 0; i < 5; i++)
        assert_equal(HsExpect[i], Hs[i]);
}
TEST(GenerateEliminationLLFactor, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";
    auto tasks = ReadTaskSet(path, "RM");
    TaskSetNormal tasksN(tasks);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination = {0, 0, 0, 1, 0};
    EliminationRecord eliminationRecord;
    eliminationRecord.Initialize(tasks.size());

    eliminationRecordGlobal.Initialize(tasks.size());
    eliminationRecordGlobal.SetEliminated(3, EliminationType::RTA);
    VectorDynamic rtaBase = RTAVector(tasks);
    MultiKeyFactor f1 = FactorGraphEnergyLL::GenerateEliminationLLFactor(tasks, 3, rtaBase(3));
    int dimension = 4;
    std::vector<MatrixDynamic> Hs, HsExpect;
    Hs.reserve(dimension);
    HsExpect.reserve(dimension);
    for (int i = 0; i < dimension; i++)
    {
        MatrixDynamic m = GenerateMatrixDynamic(1, 1);
        Hs.push_back(m);
        m << -1 * ceil(rtaBase(3) / tasks[i].period);
        HsExpect.push_back(m);
    }
    HsExpect[3] << -1;
    auto x = EnergyOptUtils::GenerateInitialFG(tasksN, eliminationRecord);
    assert_equal(GenerateVectorDynamic1D(0), f1.unwhitenedError(x, Hs), 1e-3);
    for (uint i = 0; i < HsExpect.size(); i++)
        assert_equal(HsExpect[i], Hs[i], 1e-7);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
