#include <CppUnitLite/TestHarness.h>

// #include "sources/EnergyOptimization/EnergyFactor.h"
// #include "sources/TaskModel/DAG_Task.h"
#include "sources/Utils/Parameters.h"
using namespace rt_num_opt;
// TEST(a, b) { EXPECT_LONGS_EQUAL(10000, MaxLoopControl); }
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
