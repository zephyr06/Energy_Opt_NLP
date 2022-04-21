#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/FactorGraphEnergyLL.h"
using namespace std::chrono;
using namespace rt_num_opt;
using namespace std;
TEST(EliminationRecordUpdate, v1)
{
    optimizerType = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";
    auto tasks = ReadTaskSet(path, "RM");
    VectorDynamic execCurr = GenerateVectorDynamic(5);

    execCurr << 6, 46, 50, 138, 8;
    UpdateTaskSetExecutionTime(tasks, execCurr);
    eliminationRecordGlobal.Initialize(tasks.size());
    gtsam::NonlinearFactorGraph graph = FactorGraphEnergyLL::BuildControlGraph(tasks);
    gtsam::Values initialEstimateFG = FactorGraphEnergyLL::GenerateInitialFG(tasks);
    auto sth = graph.error(initialEstimateFG);
    eliminationRecordGlobal.Print();
    EXPECT(EliminationType::Not == eliminationRecordGlobal[0].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[1].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[2].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[3].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[4].type);

    execCurr << 6, 46, 50, 138, 8.1;
    UpdateTaskSetExecutionTime(tasks, execCurr);
    eliminationRecordGlobal.Initialize(tasks.size());
    graph = FactorGraphEnergyLL::BuildControlGraph(tasks);
    initialEstimateFG = FactorGraphEnergyLL::GenerateInitialFG(tasks);
    sth = graph.error(initialEstimateFG);
    eliminationRecordGlobal.Print();
    EXPECT(EliminationType::Not == eliminationRecordGlobal[0].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[1].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[2].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[3].type);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[4].type);

    execCurr << 6, 45.9, 24, 138, 8.1;
    UpdateTaskSetExecutionTime(tasks, execCurr);
    eliminationRecordGlobal.Initialize(tasks.size());
    graph = FactorGraphEnergyLL::BuildControlGraph(tasks);
    initialEstimateFG = FactorGraphEnergyLL::GenerateInitialFG(tasks);
    sth = graph.error(initialEstimateFG);
    eliminationRecordGlobal.Print();
    EXPECT(EliminationType::Not == eliminationRecordGlobal[0].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[1].type);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[2].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[3].type);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[4].type);

    execCurr << 6, 45.9, 50.1, 500, 6;
    UpdateTaskSetExecutionTime(tasks, execCurr);
    eliminationRecordGlobal.Initialize(tasks.size());
    graph = FactorGraphEnergyLL::BuildControlGraph(tasks);
    initialEstimateFG = FactorGraphEnergyLL::GenerateInitialFG(tasks);
    sth = graph.error(initialEstimateFG);
    eliminationRecordGlobal.Print();
    EXPECT(EliminationType::Not == eliminationRecordGlobal[0].type);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[1].type);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[2].type);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[3].type);
    EXPECT(EliminationType::RTA == eliminationRecordGlobal[4].type);
}

TEST(EliminationRecord, v2)
{
    optimizerType = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";
    auto tasks = ReadTaskSet(path, "RM");
    std::vector<bool> maskForElimination(tasks.size(), false);
    eliminationRecordGlobal.Initialize(tasks.size());
    VectorDynamic rtaBase = RTALLVector(tasks);
    auto sth = EnergyOptimize::OptimizeTaskSetIterativeWeight<FactorGraphEnergyLL>(tasks);
    UpdateTaskSetExecutionTime(tasks, sth.first);
    eliminationRecordGlobal.Print();
    EnergyOptimize::FindEliminateVariableFromRecordGlobal<FactorGraphEnergyLL>(tasks);
    EXPECT(EliminationType::Bound == eliminationRecordGlobal[4].type);
}

TEST(WithEliminationRecord, overall)
{
    optimizerType = 2;
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v16.csv";
    auto tasks = ReadTaskSet(path, "RM");
    VectorDynamic vec = GenerateVectorDynamic(20);
    vec << 25, 2, 2, 52, 57, 16, 1, 86, 45, 246, 2085, 1, 1405, 604, 1866, 600, 40, 17658, 4188, 19011;
    UpdateTaskSetExecutionTime(tasks, vec);
    RTA_LL r(tasks);
    r.CheckSchedulability(true);
    eliminationRecordGlobal.Initialize(tasks.size());
    for (int i : {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11})
        eliminationRecordGlobal.SetEliminated(i, EliminationType::Bound);
    for (int i : {12, 13, 14, 15, 16, 17, 18, 19})
        eliminationRecordGlobal.SetEliminated(i, EliminationType::RTA);
    for (int i : {10})
        eliminationRecordGlobal.SetEliminated(i, EliminationType::Not);

    EnergyOptimize::FindEliminateVariableFromRecordGlobal<FactorGraphEnergyLL>(tasks);
    EXPECT(EliminationType::Not == eliminationRecordGlobal[10].type);
}
// TEST(WithEliminationRecord, overall)
// {
//     optimizerType = 2;
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v31.csv";
//     auto tasks = ReadTaskSet(path, "RM");
//     // VectorDynamic vec = GenerateVectorDynamic(20);
//     // vec << 25, 2, 2, 52, 57, 16, 1, 86, 45, 246, 2085, 1, 1405, 604, 1866, 600, 40, 17658, 4188, 19011;
//     // UpdateTaskSetExecutionTime(tasks, vec);
//     eliminationRecordGlobal.Initialize(tasks.size());
//     // for (int i : {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 15, 16})
//     //     eliminationRecordGlobal.SetEliminated(i, EliminationType::Bound);
//     // for (int i : {11, 17, 18, 19})
//     //     eliminationRecordGlobal.SetEliminated(i, EliminationType::RTA);
//     // for (int i : {12, 14})
//     //     eliminationRecordGlobal.SetEliminated(i, EliminationType::Not);

//     // eliminationRecordGlobal[0].type = EliminationType::Bound;
//     // eliminationRecordGlobal[1].type = EliminationType::Bound;
//     // eliminationRecordGlobal[2].type = EliminationType::Bound;
//     // eliminationRecordGlobal[3].type = EliminationType::Bound;
//     // eliminationRecordGlobal[4].type = EliminationType::Bound;
//     // eliminationRecordGlobal[5].type = EliminationType::Bound;
//     // eliminationRecordGlobal[6].type = EliminationType::Bound;
//     // eliminationRecordGlobal[7].type = EliminationType::Bound;
//     // eliminationRecordGlobal[8].type = EliminationType::Bound;
//     // eliminationRecordGlobal[9].type = EliminationType::Bound;
//     // eliminationRecordGlobal[10].type = EliminationType::Bound;
//     // eliminationRecordGlobal[11].type = EliminationType::RTA;
//     // eliminationRecordGlobal[12].type = EliminationType::Not;
//     // eliminationRecordGlobal[13].type = EliminationType::Bound;
//     // eliminationRecordGlobal[14].type = EliminationType::Not;
//     // eliminationRecordGlobal[15].type = EliminationType::Bound;
//     // eliminationRecordGlobal[16].type = EliminationType::Bound;
//     // eliminationRecordGlobal[17].type = EliminationType::RTA;
//     // eliminationRecordGlobal[18].type = EliminationType::RTA;
//     // eliminationRecordGlobal[19].type = EliminationType::RTA;

//     // EnergyOptimize::FindEliminateVariableFromRecordGlobal<FactorGraphEnergyLL>(tasks);
//     auto sth = EnergyOptimize::OptimizeTaskSetIterative<FactorGraphEnergyLL>(tasks);
// }
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
