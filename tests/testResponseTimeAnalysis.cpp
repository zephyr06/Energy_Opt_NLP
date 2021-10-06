#include <CppUnitLite/TestHarness.h>
#include "../sources/RTA_LL.h"
#include "../sources/RTA_WAP.h"

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
    CHECK_EQUAL(rta3Expect, RTA_LL::ResponseTimeAnalysisWarm(rta3Expect - 100, task_set[2], hp));
}
TEST(RTA, RTA1)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    int rta3Actual = RTA_LL::ResponseTimeAnalysis(task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
}
TEST(RTA, RTA2)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta2Expect = 265;
    TaskSet hp2({task_set[0]});
    int rta2Actual = RTA_LL::ResponseTimeAnalysis(task_set[1], hp2);
    CHECK_EQUAL(rta2Expect, rta2Actual);

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = RTA_LL::ResponseTimeAnalysis(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}
TEST(RTA, RTA3)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = RTA_LL::ResponseTimeAnalysis(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}

TEST(RTA, ResponseTimeAnalysisWarm)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    double delta = 1e-4;
    double rta3Expect = 282 + delta * 1;
    TaskSet hp({task_set[0], task_set[1]});
    task_set[2].executionTime += delta;
    double rta3Actual = RTA_LL::ResponseTimeAnalysisWarm(rta3Expect - 100, task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
    cout << "RTA "
         << "ResponseTimeAnalysisWarm"
         << " passed\n";
}

// TEST(Schedulability, p1)
// {
//     string path2 = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
//     auto task_set = ReadTaskSet(path2, "RM");
//     bool schedulable = CheckSchedulability<RTA_LL>(task_set);
//     if (not schedulable)
//     {
//         cout << "The test set in schedulablability test didn't pass!\n";
//         throw;
//     }
// }
TEST(RTA, ResponseTimeOfTaskSetHard)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    int rta3Actual = int(ResponseTimeOfTaskSetHard<RTA_LL>(task_set)(2, 0));
    CHECK_EQUAL(rta3Expect, rta3Actual);
}

TEST(wap, v1)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
    A_Global = GenerateZeroMatrix(5, 5);
    P_Global = GenerateOneMatrix(5, 5);

    VectorDynamic expect;
    expect.resize(5, 1);
    expect << 10, 21, 33, 46, 60;
    VectorDynamic actual = ResponseTimeOfTaskSetHard<RTA_WAP>(task_set);
    AssertEigenEqualVector(expect, actual);
}

TEST(wap, v2)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
    P_Global = GenerateZeroMatrix(5, 5);
    A_Global = GenerateOneMatrix(5, 5);

    VectorDynamic expect;
    expect.resize(5, 1);
    expect << 10, 31, 55, 82, 112;
    VectorDynamic actual = ResponseTimeOfTaskSetHard<RTA_WAP>(task_set);
    AssertEigenEqualVector(expect, actual);
}

TEST(wap, v3)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v10.csv", "orig");
    A_Global = GenerateZeroMatrix(5, 5);
    A_Global << 0, 1, 0, 1, 0,
        1, 0, 1, 0, 1,
        0, 1, 0, 1, 0,
        1, 0, 1, 0, 1,
        0, 0, 0, 0, 0;
    P_Global = GenerateZeroMatrix(5, 5);
    P_Global << 1, 0, 1, 0, 1,
        0, 1, 0, 1, 0,
        1, 0, 1, 0, 1,
        0, 1, 0, 1, 0,
        1, 1, 1, 1, 1;

    VectorDynamic expect;
    expect.resize(5, 1);
    expect << 10, 31, 54, 81, 110;
    VectorDynamic actual = ResponseTimeOfTaskSetHard<RTA_WAP>(task_set);
    AssertEigenEqualVector(expect, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
