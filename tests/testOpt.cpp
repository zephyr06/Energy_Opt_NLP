#include <CppUnitLite/TestHarness.h>

#include "../sources/Optimize.h"

TEST(UpdateTaskSetExecutionTime, a1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");

    ComputationTimeVector comp;
    comp << 10, 20, 30;
    UpdateTaskSetExecutionTime(taskSet1, comp);
    CHECK_EQUAL(10, taskSet1[0].executionTime);
}

TEST(ComputationFactor, a1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");
    cout << OptimizeTaskSet(taskSet1) << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
