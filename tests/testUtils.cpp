

#include <CppUnitLite/TestHarness.h>

#include "../sources/utils.h"

TEST(ReadTaskSet, p1)
{
    cout << "Enter test\n"
         << endl;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";

    TaskSet taskset1 = ReadTaskSet(path, "RM");

    CHECK_EQUAL(500, taskset1.tasks[0].period);
    CHECK_EQUAL(550, taskset1.tasks[1].period);
    CHECK_EQUAL(880, taskset1.tasks[2].period);
    CHECK_EQUAL(2, taskset1.tasks[0].overhead);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
