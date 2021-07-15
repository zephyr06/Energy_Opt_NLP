#include <chrono>

#include <CppUnitLite/TestHarness.h>

#include "../sources/Optimize.h"
using namespace std::chrono;
/**
TEST(FindTaskDoNotNeedOptimize, A1)
{
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");

    int N = tasks.size();
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard(tasks);
    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;

    int indexExpect = 1;
    int indexActual = FindTaskDoNotNeedOptimize(tasks, initialExecutionTime, 0, responseTimeInitial);
    CHECK_EQUAL(indexExpect, indexActual);
}
TEST(NumericalDerivativeDynamic, A1)
{
    // NumericalDerivativeDynamic
    // f: R2 -> R4

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
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
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
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    int N = taskSet1.size();

    int lastTaskDoNotNeedOptimize = 1;

    VectorDynamic initialEstimate;
    initialEstimate.resize(numberOfTasksNeedOptimize, 1);
    initialEstimate << 62;

    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard(taskSet1);
    vectorGlobalOpt.resize(N, 1);
    VectorDynamic res1 = UnitOptimization(taskSet1, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    // 204 corresponds to RT of 319, which should be the best we can get because of the clamp function
    if (not(abs(204 - res1(0, 0)) < 5))
        throw;
}

TEST(OptimizeTaskSet, a1)
{
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    double res = OptimizeTaskSet(taskSet1);
    cout << "The energy saving ratio is " << res << endl;
    if (not assert_equal<double>(0.71, res, 0.01))
        throw;
}

TEST(checkConvergenceInterior, a1)
{
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
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    VectorDynamic initialExecution = GetParameterVD<int>(taskSet1, "executionTime");
    eliminateTol = 3;
    int index = FindTaskDoNotNeedOptimize(taskSet1, initialExecution, 0, initialExecution, 1);
    if (index != 0)
    {
        throw;
    }
    eliminateTol = 198;
    index = FindTaskDoNotNeedOptimize(taskSet1, initialExecution, 0, initialExecution, 1);
    if (index != 2)
        throw;
}
TEST(UnitOptimizationIPM, a1)
{
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v21.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    VectorDynamic initialExecution = GetParameterVD<int>(tasks, "executionTime");
    eliminateTol = 1;
    enableIPM = 1;
    VectorDynamic initial;
    initial.resize(1, 1);
    initial << initialExecution(2, 0);
    VectorDynamic res = UnitOptimizationIPM(tasks, 1, initial, initialExecution, initialExecution);
    cout << "In unit test UnitOptimizationIPM, the res is " << res << endl;
    if (abs(res(0, 0) - 230) < 0.1)
    {
        throw;
    }
}
*/
TEST(OptimizeTaskSet, OptimizeTaskSetOneIte)
{
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";
    cout << endl
         << path << endl
         << endl;
    TaskSet taskSet1 = ReadTaskSet(path, "utilization");
    auto start = chrono::high_resolution_clock::now();
    double res = OptimizeTaskSet(taskSet1);
    if (not assert_equal<double>(0.295, res, 0.2))
        throw;
    cout << "The energy saving ratio is " << res << endl;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << endl;
}
// TEST(OptimizeTaskSet, a2)
// {
//     // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
//     string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";
//     // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
//     // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
//     // string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

//     TaskSet taskSet1 = ReadTaskSet(path, readTaskMode);
//     auto start = chrono::high_resolution_clock::now();
//     double res = OptimizeTaskSet(taskSet1);
//     cout << blue << "The energy saving ratio is " << res << def << endl;
//     auto stop = chrono::high_resolution_clock::now();
//     auto duration = duration_cast<microseconds>(stop - start);
//     cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << endl;
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
