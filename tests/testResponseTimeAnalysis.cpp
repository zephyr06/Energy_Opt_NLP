#include <CppUnitLite/TestHarness.h>
#include "../sources/ResponseTimeAnalysis.h"

TEST(hyperPeriod, RTA)
{
    auto task_set = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv", "orig");
    int periodActual = hyperPeriod(task_set);
    int periodExpect = 1278900;
    CHECK_EQUAL(periodExpect, periodActual);

    int rta3Expect = 282;
    TaskSet hp({task_set[0], task_set[1]});
    int rta3Actual = ResponseTimeAnalysis(task_set[2], hp);
    CHECK_EQUAL(rta3Expect, rta3Actual);
    CHECK_EQUAL(rta3Expect, ResponseTimeAnalysisWarm(rta3Expect - 100, task_set[2], hp));

    int rta2Expect = 265;
    TaskSet hp2({task_set[0]});
    int rta2Actual = ResponseTimeAnalysis(task_set[1], hp2);
    CHECK_EQUAL(rta2Expect, rta2Actual);

    int rta1Expect = 12;
    TaskSet hp3({});
    int rta1Actual = ResponseTimeAnalysis(task_set[0], hp3);
    CHECK_EQUAL(rta1Expect, rta1Actual);
}

// TEST(TaskSet, ResponseTime)
// {

// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
