#include "../sources/LLBatchCompare.h"
TEST(parameters, a1)
{
    BatchCompare<Task, RTA_LL>();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
