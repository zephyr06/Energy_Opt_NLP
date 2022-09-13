#include "sources/EnergyOptimization/Energy.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace std;
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

    AssertEqualScalar(0.9 / 1.9, GetFrequency(tasks[0]), 1e-6, __LINE__);
}
TEST(Jacobian, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N3.csv";
    auto tasks = ReadTaskSet(path, "RM");
    executionTimeModel = 2;
    EnergyMode = 2;
}
TEST(ReadYecheng, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v18.csv";
    auto tasks = ReadTaskSet(path, "RM");
}
// TEST(EstimateEnergyTaskSet, v1)
// {
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v12.csv";
//     auto tasks = ReadTaskSet(path, "RM");
//     executionTimeModel = 1;
//     EnergyMode = 1;
//     VectorDynamic comp;
//     comp.resize(tasks.size(), 1);
//     comp << 1.75805, 1.75805, 1.75805, 3.5161, 1.75805,
//         1.75805, 2.74945, 2.74945, 6.27055, 3.76233,
//         3.76233, 5.01644, 4.79876, 1.19969, 31.6201,
//         1.17112, 11.7112, 3.51335, 9.36892, 40.0195;
//     UpdateTaskSetExecutionTime(tasks, comp);
//     auto actual = EstimateEnergyTaskSet(tasks);
//     cout << actual << endl;
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
