#include <CppUnitLite/TestHarness.h>
#include "../sources/ResponseTimeAnalysis.h"

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
    CHECK_EQUAL(rta3Expect, ResponseTimeAnalysisWarm<int>(rta3Expect - 100, task_set[2], hp));
}
TEST(RTA, RTA1)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    int rta3Actual = ResponseTimeAnalysis<int>(task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
}
TEST(RTA, RTA2)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta2Expect = 265;
    TaskSet hp2({task_set[0]});
    int rta2Actual = ResponseTimeAnalysis<int>(task_set[1], hp2);
    CHECK_EQUAL(rta2Expect, rta2Actual);

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = ResponseTimeAnalysis<int>(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}
TEST(RTA, RTA3)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = ResponseTimeAnalysis<int>(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}

TEST(RTA, float)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    float delta = 1e-4;
    float rta3Expect = 282 + delta * 1;
    TaskSet hp({task_set[0], task_set[1]});
    task_set[2].executionTime += delta;
    CHECK_EQUAL(rta3Expect, ResponseTimeAnalysisWarm<float>(rta3Expect - 100, task_set[2], hp));
}
// TEST(RTA, float2)
// {
//     auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
//     float delta = 1e-8;
//     float rta3Expect = 282 + delta * 2;
//     TaskSet hp({task_set[0], task_set[1]});
//     task_set[2].executionTime += delta;
//     DOUBLES_EQUAL(rta3Expect, ResponseTimeAnalysisWarm<float>(rta3Expect - 100, task_set[2], hp), delta / 10);
// }

TEST(Schedulability, p1)
{
    string path2 = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    bool schedulable = CheckSchedulability<int>(ReadTaskSet(path2, "RM"));
    if (not schedulable)
    {
        cout << "The test set in schedulablability test didn't pass!\n";
        throw;
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
