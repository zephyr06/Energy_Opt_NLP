#include "../sources/Energy.h"
#include "../sources/Declaration.h"
#include "../sources/testMy.h"

TEST(energy, tasks)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto tasks = ReadTaskSet(path, "RM");
    executionTimeModel = 1;
    EnergyMode = 1;
    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17, 12, 253;
    UpdateTaskSetExecutionTime(tasks, comp);

    VectorDynamic energy1;
    weightEnergy = 1e8;
    energy1 = EstimateEnergyTaskSet(tasks);
    AssertEqualScalar(3777777.7778, energy1(0, 0));
    AssertEqualScalar(2448979.5918, energy1(1, 0));
}

TEST(GetFrequency, tasks)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto tasks = ReadTaskSet(path, "RM");
    executionTimeModel = 2;
    EnergyMode = 1;
    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17, 12, 253;
    UpdateTaskSetExecutionTime(tasks, comp);

    AssertEqualScalar(1, GetFrequency(tasks[0]));
    AssertEqualScalar(1, GetFrequency(tasks[1]));
}

TEST(GetFrequency, tasks2)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto tasks = ReadTaskSet(path, "RM");
    executionTimeModel = 2;
    EnergyMode = 1;
    VectorDynamic comp;
    comp.resize(3, 1);
    comp << 17 * 2, 12, 253;
    UpdateTaskSetExecutionTime(tasks, comp);

    AssertEqualScalar(0.9 / 1.9, GetFrequency(tasks[0]));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
