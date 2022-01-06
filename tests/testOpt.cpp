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
    computationBound = 100;

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    int N = tasks.size();
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard<RTA_LL>(tasks);
    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;

    int indexExpect = 1;
    int indexActual = Opt_LL::FindTaskDoNotNeedOptimize(tasks, initialExecutionTime, 0, responseTimeInitial);
    CHECK_EQUAL(indexExpect, indexActual);
}
TEST(NumericalDerivativeDynamic, A1)
{
    // NumericalDerivativeDynamic
    // f: R2 -> R4
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
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
    if (not assert_equal(H_Expect, H_Actual, 1e-3))
        throw;
}

TEST(UpdateTaskSetExecutionTime, A1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    VectorDynamic executionTimeVec;
    executionTimeVec.resize(1, 1);
    executionTimeVec << 80;
    UpdateTaskSetExecutionTime(taskSet1, executionTimeVec, 1);
    if (not(80 == taskSet1[2].executionTime))
        throw;
}

TEST(unitOptimization, a1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    int N = taskSet1.size();
    optimizerType = 1;
    EnergyMode = 1;
    int lastTaskDoNotNeedOptimize = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize(numberOfTasksNeedOptimize, 1);
    initialEstimate << 62;

    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard<RTA_LL>(taskSet1);
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
    computationBound = 100;
    optimizerType = 1;
    EnergyMode = 1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    computationBound = 100;
    executionTimeModel = 1;
    double res = Opt_LL::OptimizeTaskSet(taskSet1);
    cout << "The energy saving ratio is " << res << endl;
    if (not assert_equal<double>(0.359257, res, 0.01))
        CoutWarning("One test case failed in performance!");
}

TEST(checkConvergenceInterior, a1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
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
TEST(FindTaskDoNotNeedOptimize, a1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    optimizerType = 1;
    EnergyMode = 1;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    VectorDynamic initialExecution = GetParameterVD<int>(taskSet1, "executionTime");
    double eliminateTol_t = eliminateTol;
    eliminateTol = 3;
    int index = Opt_LL::FindTaskDoNotNeedOptimize(taskSet1, initialExecution, 0, initialExecution, 1);
    if (index != 0)
    {
        throw;
    }
    eliminateTol = 198;
    index = Opt_LL::FindTaskDoNotNeedOptimize(taskSet1, initialExecution, 0, initialExecution, 1);
    if (index != 2)
        throw;
    eliminateTol = eliminateTol_t;
}

TEST(OptimizeTaskSetOneIte, a2)
{
    enableMaxComputationTimeRestrict = 1;
    computationBound = 2;
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";
    cout << endl
         << path << endl
         << endl;
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    minWeightToBegin = 1e3;
    optimizerType = 1;
    EnergyMode = 1;
    executionTimeModel = 1;
    double res = Opt_LL::OptimizeTaskSet(taskSet1);
    AssertEqualScalar(0.12, res, 0.04, __LINE__);
    if (not assert_equal<double>(0.12, res, 0.02))
        CoutWarning("One test case failed in performance!");
    cout << "The energy saving ratio in OptimizeTaskSet-OptimizeTaskSetOneIte is " << res << endl;
}

TEST(ClampComputationTime, a1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    int N = taskSet1.size();

    int lastTaskDoNotNeedOptimize = 1;
    eliminateTol = 10;
    optimizerType = 1;
    EnergyMode = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize(numberOfTasksNeedOptimize, 1);
    initialEstimate << 62;

    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard<RTA_LL>(taskSet1);
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

TEST(ClampComputationTime, a2)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v22.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");

    // double res = Opt_LL::OptimizeTaskSet(taskSet1);
    // CoutWarning("One test case failed in performance!");
}

TEST(UnitOptimizationIPM, a1)
{
    enableMaxComputationTimeRestrict = 0;
    computationBound = 100;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v21.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    VectorDynamic initialExecution = GetParameterVD<int>(tasks, "executionTime");
    eliminateTol = 1;
    optimizerType = 1;
    EnergyMode = 1;
    // enableIPM = 1;
    vectorGlobalOpt.resize(3, 1);
    VectorDynamic initial;
    initial.resize(1, 1);
    initial << initialExecution(2, 0);
    VectorDynamic res = Opt_LL::UnitOptimizationIPM(tasks, tasks, 1, initial, initialExecution, initialExecution);
    cout << "In unit test UnitOptimizationIPM, the res is " << res << endl;
    AssertEqualScalar(230, res(0, 0), 1.1, __LINE__);
    if (not(abs(res(0, 0) - 230) < 0.1))
    {
        cout << "Error in UnitOptimizationIPM-a1" << endl;
        CoutWarning("One test case failed in performance!");
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
