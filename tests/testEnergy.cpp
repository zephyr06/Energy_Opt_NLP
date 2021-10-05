#include "../sources/Energy.h"
#include "../sources/Declaration.h"
#include "../sources/testMy.h"

TEST(energy, tasks)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto taskSet = ReadTaskSet(path, "RM");

    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17, 12, 253;

    VectorDynamic energy1;
    weightEnergy = 1e8;
    energy1 = EstimateEnergyTaskSet(taskSet, comp);
    AssertEqualScalar(3777777.7778, energy1(0, 0));
    AssertEqualScalar(2448979.5918, energy1(1, 0));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
