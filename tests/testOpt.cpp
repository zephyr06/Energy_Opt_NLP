#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
using namespace std::chrono;
using Opt_LL = Energy_Opt<RTA_LL>;

// There are two types of tests, strict deadline test, or period & 2xExecution test

TEST(FindTaskDoNotNeedOptimize, A1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    InitializeGlobalVector(tasks.size());
    int N = tasks.size();
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;

    int indexExpect = 1;
    int indexActual = Opt_LL::FindTaskDoNotNeedOptimize(tasks, 0, responseTimeInitial, eliminateTol);
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
    boost::function<Matrix(const VectorDynamic &)> f =
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
    int N = taskSet1.size();
    InitializeGlobalVector(taskSet1.size());
    optimizerType = 1;
    EnergyMode = 1;
    int lastTaskDoNotNeedOptimize = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize((N - lastTaskDoNotNeedOptimize - 1), 1);
    initialEstimate << 62;

    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(taskSet1);
    vectorGlobalOpt.resize(N, 1);
    VectorDynamic res1 = Opt_LL::UnitOptimization(taskSet1, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
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
    InitializeGlobalVector(taskSet1.size());

    double res = Opt_LL::OptimizeTaskSet(taskSet1);
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
    double res = Opt_LL::OptimizeTaskSet(taskSet1);
    AssertEqualScalar(0.28, res, 0.04, __LINE__);
    if (not assert_equal<double>(0.28, res, 0.02))
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

    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(taskSet1);
    vectorGlobalOpt.resize(N, 1);
    VectorDynamic res1 = Opt_LL::UnitOptimization(taskSet1, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << res1 << endl;
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

    VectorDynamic initialEstimate;
    initialEstimate.resize(N, 1);
    initialEstimate << 15.2, 13.1, 12.1, 16.2, 19.5;
    UpdateTaskSetExecutionTime(tasks, initialEstimate);
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    Opt_LL::ClampComputationTime(tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, "fine");
    VectorDynamic expect1 = initialEstimate;
    expect1 << 15, 13, 17, 24, 19;
    EXPECT(assert_equal(expect1, GetParameterVD<double>(tasks, "executionTime")));
}
TEST(ClampComputationTime, v2)
{
    enableMaxComputationTimeRestrict = 1;
    MaxComputationTimeRestrict = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v22.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    InitializeGlobalVector(tasks.size());
    int N = tasks.size();

    int lastTaskDoNotNeedOptimize = -1;
    eliminateTol = 0.1;
    optimizerType = 1;
    EnergyMode = 1;
    MaxComputationTimeRestrict = 2;

    VectorDynamic initialEstimate;
    initialEstimate.resize(N, 1);
    initialEstimate << 15.2, 13.1, 12.1, 16.2, 19.5;
    UpdateTaskSetExecutionTime(tasks, initialEstimate);
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    Opt_LL::ClampComputationTime(tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, "fine");
    VectorDynamic expect1 = initialEstimate;
    expect1 << 20, 22, 24, 26, 28;
    EXPECT(assert_equal(expect1, GetParameterVD<double>(tasks, "executionTime")));
}
TEST(UnitOptimizationIPM, a1)
{
    enableMaxComputationTimeRestrict = 0;
    MaxComputationTimeRestrict = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v21.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
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
    VectorDynamic res = Opt_LL::UnitOptimization(tasks, 1, initial, initialExecution);
    cout << "In unit test UnitOptimizationIPM, the res is " << res << endl;
    AssertEqualScalar(230, res(0, 0), 1.1, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
