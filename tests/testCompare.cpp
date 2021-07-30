#include "../sources/BatchCompare.h"

TEST(ReadBaselineResult, V1)
{
    string path = "forTestperiodic-set-0-syntheticJobs.csv";
    auto sth = ReadBaselineResult(path, 5);
    if (sth.first != 0.0255572)
    {
        cout << "ReadBaselineResult-v1 error!" << endl;
        throw;
    }
    if (sth.second != 6.14617e+08)
    {
        cout << "ReadBaselineResult-v1 error!" << endl;
        throw;
    }
}

TEST(parameters, a1)
{
    BatchCompare();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
