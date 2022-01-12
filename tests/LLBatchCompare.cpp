#include "../sources/LLBatchCompare.h"
TEST(parameters, a1)
{
    BatchCompare();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
