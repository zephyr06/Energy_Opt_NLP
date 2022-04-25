#include <CppUnitLite/TestHarness.h>

#include "sources/TaskModel/DAG_Task.h"
#include <dagSched/DAGTask.h>
// #include "DAG-scheduling_Verucchi/include/dagSched/DAGTask.h"

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
