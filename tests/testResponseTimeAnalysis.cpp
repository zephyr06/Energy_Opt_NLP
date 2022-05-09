#include <CppUnitLite/TestHarness.h>
#include "sources/RTA/RTA_LL.h"
#include "sources/Tools/testMy.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/ControlOptimization/ControlOptimize.h"

using namespace rt_num_opt;
using namespace std;
TEST(hyperPeriod, RTA)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    long long int periodActual = HyperPeriod(task_set);
    int periodExpect = 1278900;
    CHECK_EQUAL(periodExpect, periodActual);
}
TEST(RTA, RTA0)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    RTA_LL r(task_set);
    CHECK_EQUAL(rta3Expect, r.ResponseTimeAnalysisWarm(rta3Expect - 100, task_set[2], hp));
}
TEST(RTA, RTA1)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    RTA_LL r(task_set);
    int rta3Actual = r.ResponseTimeAnalysis(task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
}
TEST(RTA, RTA2)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta2Expect = 265;
    TaskSet hp2({task_set[0]});
    RTA_LL r(task_set);
    int rta2Actual = r.ResponseTimeAnalysis(task_set[1], hp2);
    CHECK_EQUAL(rta2Expect, rta2Actual);

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = r.ResponseTimeAnalysis(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}
TEST(RTA, RTA3)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    RTA_LL r(task_set);
    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = r.ResponseTimeAnalysis(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}
TEST(RTA, RTA4)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
    RTA_LL r(task_set);
    TaskSetNormal tasks(task_set);
    VectorDynamic expect = GenerateVectorDynamic(5);
    expect << 10, 21, 33, 46, 60;
    VectorDynamic actual = r.ResponseTimeOfTaskSet();
    AssertEigenEqualVector(expect, actual);
}

// TEST(GetBusyPeriod, v1)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     int N = task_set.size();
//     int busyExpect = 74;
//     MatrixDynamic A = GenerateMatrixDynamic(N, N);
//     MatrixDynamic P = GenerateMatrixDynamic(N, N);
//     int busyActual = RTA_WAP::GetBusyPeriod(task_set, A, P, 4);
//     AssertEqualScalar(busyExpect, busyActual);
// }

// TEST(GetBusyPeriod, v2)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     int N = task_set.size();
//     int busyExpect = 74;
//     MatrixDynamic A = GenerateMatrixDynamic(N, N);
//     MatrixDynamic P = GenerateMatrixDynamic(N, N);
//     P << 1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 1, 1, 1, 1;
//     int busyActual = RTA_WAP::GetBusyPeriod(task_set, A, P, 4);
//     AssertEqualScalar(busyExpect, busyActual);
// }

// TEST(GetBusyPeriod, v3)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v16.csv", "orig");
//     int N = task_set.size();
//     int busyExpect = 80;
//     MatrixDynamic A = GenerateMatrixDynamic(N, N);
//     MatrixDynamic P = GenerateMatrixDynamic(N, N);
//     P << 1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 1, 1, 1, 1;
//     int busyActual = RTA_WAP::GetBusyPeriod(task_set, A, P, 4);
//     AssertEqualScalar(busyExpect, busyActual);
// }

TEST(RTA, ResponseTimeAnalysisWarm)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    double delta = 1e-4;
    double rta3Expect = 282 + delta * 1;
    TaskSet hp({task_set[0], task_set[1]});
    task_set[2].executionTime += delta;
    RTA_LL r(task_set);
    double rta3Actual = r.ResponseTimeAnalysisWarm(rta3Expect - 100, task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
    std::cout << "RTA "
              << "ResponseTimeAnalysisWarm"
              << " passed\n";
}

TEST(RTA_LL, v1)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n30_v2.csv", "RM");
    TaskSetNormal tasks(task_set);
    RTA_LL r(task_set);
    AssertBool(true, r.CheckSchedulability(true));
}

TEST(RTA, ResponseTimeOfTaskSet)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    TaskSetNormal tasks(task_set);
    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    RTA_LL r(task_set);
    int rta3Actual = int(r.ResponseTimeOfTaskSet()(2, 0));
    CHECK_EQUAL(rta3Expect, rta3Actual);
}

// TEST(wap, v1)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     A_Global = GenerateMatrixDynamic(5, 5);
//     P_Global = GenerateOneMatrix(5, 5);
//     TaskSetNormal tasks(task_set);
//     VectorDynamic expect;
//     expect.resize(5, 1);
//     expect << 10, 21, 33, 46, 60;
//     RTA_WAP r(task_set);
//     VectorDynamic actual = r.ResponseTimeOfTaskSet();
//     AssertEigenEqualVector(expect, actual);
// }

// TEST(wap, v2)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     P_Global = GenerateMatrixDynamic(5, 5);
//     A_Global = GenerateOneMatrix(5, 5);

//     VectorDynamic expect;
//     expect.resize(5, 1);
//     expect << 10, 31, 55, 82, 112;
//     RTA_WAP r(task_set);
//     VectorDynamic actual = r.ResponseTimeOfTaskSet();
//     AssertEigenEqualVector(expect, actual);
// }

// TEST(wap, v3)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     A_Global = GenerateMatrixDynamic(5, 5);
//     A_Global << 0, 1, 0, 1, 0,
//         1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 0, 1, 0, 1,
//         0, 0, 0, 0, 0;
//     P_Global = GenerateMatrixDynamic(5, 5);
//     P_Global << 1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 0, 1, 0, 1,
//         0, 1, 0, 1, 0,
//         1, 1, 1, 1, 1;

//     VectorDynamic expect;
//     expect.resize(5, 1);
//     expect << 10, 31, 54, 81, 110;
//     RTA_WAP r(task_set);
//     VectorDynamic actual = r.ResponseTimeOfTaskSet();
//     AssertEigenEqualVector(expect, actual);
// }

// TEST(GenerateWAP, v1)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(true, success);
//     AssertEigenEqualMatrix(expectA, actualA);
//     AssertEigenEqualMatrix(expectA, A_Global);
//     AssertEigenEqualMatrix(expectP, P_Global);
// }
// TEST(GenerateWAP, v2)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v11.csv", "RM");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     expectP << 0, 0, 0, 0, 0,
//         0, 0, 0, 0, 1,
//         0, 0, 0, 0, 1,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0;
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(true, success);
//     if (success)
//     {
//         AssertEigenEqualMatrix(expectA, A_Global);
//         AssertEigenEqualMatrix(expectP, P_Global);
//     }
// }
// TEST(GenerateWAP, v3)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v5.csv", "orig");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     expectP << 0, 1, 1, 1, 0,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0;
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(false, success);
//     if (success)
//     {
//         AssertEigenEqualMatrix(expectA, A_Global);
//         AssertEigenEqualMatrix(expectP, P_Global);
//     }
// }
// TEST(GenerateWAP, v4)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v12.csv", "RM");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     expectP << 0, 1, 0, 1, 1,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 1, 1,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0;
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(true, success);
//     if (success)
//     {
//         AssertEigenEqualMatrix(expectA, A_Global);
//         AssertEigenEqualMatrix(expectP, P_Global);
//     }
// }
// TEST(GenerateWAP, v5)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v13.csv", "RM");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     expectA(2, 4) = 1;
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     expectP << 0, 1, 0, 1, 1,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0,
//         0, 0, 0, 0, 0;
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(true, success);
//     if (success)
//     {
//         AssertEigenEqualMatrix(expectA, A_Global);
//         AssertEigenEqualMatrix(expectP, P_Global);
//     }
// }
// TEST(GenerateWAP, v6)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v15.csv", "RM");
//     int N = task_set.size();
//     MatrixDynamic expectA = GenerateMatrixDynamic(N, N);
//     // expectA(2, 4) = 1;
//     MatrixDynamic expectP = GenerateMatrixDynamic(N, N);
//     expectP << 0, 1, 0, 1, 1,
//         0, 0, 0, 1, 1,
//         0, 0, 0, 1, 1,
//         0, 0, 0, 0, 1,
//         0, 0, 0, 0, 0;
//     auto sth = Generate_WAP(task_set);
//     MatrixDynamic actualA, actualP;
//     bool success;
//     std::tie(success, actualA, actualP) = sth;
//     AssertBool(true, success);
//     if (success)
//     {
//         AssertEigenEqualMatrix(expectA, A_Global);
//         AssertEigenEqualMatrix(expectP, P_Global);
//     }
// }
// TO BE FIXED!
// TEST(dag, v1)
// {
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v28.csv";

//     auto dagTasks = ReadDAG_Tasks(path, "orig");
//     RTA_DAG rtaDag(dagTasks);
//     core_m_dag = 1;
//     double expect = 17;
//     double actual = rtaDag.RTA_Common_Warm(0, 0);
//     AssertEqualScalar(expect, actual);
// }
// TEST(dag, v2)
// {
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v28.csv";

//     auto dagTasks = ReadDAG_Tasks(path, "orig");
//     RTA_DAG rtaDag(dagTasks);

//     core_m_dag = 2;

//     double expect = 14;
//     double actual = rtaDag.RTA_Common_Warm(0, 0);
//     AssertEqualScalar(expect, actual);

//     expect = 15.5;
//     actual = rtaDag.RTA_Common(1);
//     AssertEqualScalar(expect, actual, 1e-1, __LINE__);
// }
TEST(LL, vn)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    periodInitial1 << 127, 136, 233, 127, 122;
    UpdateTaskSetPeriod(tasks, periodInitial1);
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    AssertEqualScalar(INT32_MAX, rta(4, 0));
}
TEST(LL, vn1)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(5);
    periodInitial1 << 57.513,
        311.526,
        393.204,
        119,
        298.655;
    UpdateTaskSetPeriod(tasks, periodInitial1);
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    AssertEqualScalar(119, rta(3, 0));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
