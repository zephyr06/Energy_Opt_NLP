#include <CppUnitLite/TestHarness.h>

#include "sources/RTA/RTA_Narsi19.h"
TEST()
{
    std::vector<int> Ns = {3, 4, 5, 6, 7, 8, 9, 10};
    for (int N : Ns)
    {
        std::string outDirectory = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" + to_string(N) + "/";
    }
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
