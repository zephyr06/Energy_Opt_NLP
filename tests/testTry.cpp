#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;

// There are two types of tests, strict deadline test, or period & 2xExecution test

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
