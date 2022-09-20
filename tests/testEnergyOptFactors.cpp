#include "sources/EnergyOptimization/EnergyOptimize.h"
#include "sources/TaskModel/DAG_Nasri19.h"
#include "sources/RTA/RTA_Nasri19.h"

#include <CppUnitLite/TestHarness.h>
using namespace rt_num_opt;
TEST(UnitOpt, v1)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".yaml";
    if (optimizerType >= 5)
        optimizerType = 2;
    rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
    InitializeGlobalVector(tasksN.tasks_.size());
    AssertBool(true, tasksN.tasks_.size() > 0, __LINE__);

    EliminationRecord eliminationRecordGlobal;
    eliminationRecordGlobal.Initialize(tasksN.size());
    InitializeGlobalVector(tasksN.size());

    auto resPair = Energy_OptDAG<rt_num_opt::DAG_Nasri19, RTA_Nasri19>::UnitOptimization(tasksN, eliminationRecordGlobal);
    VectorDynamic exec = resPair.first;
    UpdateTaskSetExecutionTime(tasksN, exec);
    RTA_Nasri19 rr(tasksN);
    std::cout << rr.CheckSchedulability();
    std::cout << Color::blue << "The result is " << resPair.second << Color::def << std::endl;
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}