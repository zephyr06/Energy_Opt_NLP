

#include <CppUnitLite/TestHarness.h>

#include "../sources/Tasks.h"
#include "../sources/ResponseTimeAnalysis.h"
#include "../sources/Parameters.h"

TEST(ReadTaskSet, p1)
{

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_n3_v1.csv";

    TaskSet taskset1 = ReadTaskSet(path, "RM");

    CHECK_EQUAL(500, taskset1[0].period);
    CHECK_EQUAL(550, taskset1[1].period);
    CHECK_EQUAL(880, taskset1[2].period);
    CHECK_EQUAL(2, taskset1[0].overhead);
}
TEST(parameters, a1)
{
    if (TASK_NUMBER == 0)
        throw;
    cout << "The task number is " << TASK_NUMBER << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
