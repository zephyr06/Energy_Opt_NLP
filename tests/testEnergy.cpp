#include "../sources/Energy.h"

TEST(energy, tasks)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto taskSet = ReadTaskSet(path, "RM");

    vector<float> energy = EstimateEnergyTaskSet(taskSet, {1, 1, 1});
    CHECK_EQUAL(48314, energy[0]);
    CHECK_EQUAL(31320, energy[1]);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
