#include "sources/LLBatchCompare.h"

using namespace rt_num_opt;
TEST(parameters, a1)
{
    BatchCompare();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
