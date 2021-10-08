#include "../sources/WAPbatchOptimize.h"
TEST(parameters, a1)
{
    BatchOptimize();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
