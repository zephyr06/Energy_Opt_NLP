#include "../sources/Energy.h"
#include "../sources/Declaration.h"

TEST(energy, tasks)
{
    string path = "/home/lab/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto taskSet = ReadTaskSet(path, "RM");

    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17, 12, 253;

    VectorDynamic energy1;
    energy1 = EstimateEnergyTaskSet(taskSet, comp);
    CHECK_EQUAL(48314, energy1(0, 0));
    CHECK_EQUAL(31320, energy1(1, 0));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
