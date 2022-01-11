#include "../sources/WAPbatchOptimize.h"
TEST(parameters, a1)
{
    BatchOptimize<Task, RTA_WAP>();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}