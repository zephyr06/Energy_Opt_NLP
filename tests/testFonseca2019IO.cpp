#include <CppUnitLite/TestHarness.h>

#include "sources/Utils/Parameters.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/BatchTestutils.h"

TEST(addDummy, v1)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_dag_n10_v1.csv";
    auto dagTasks = rt_num_opt::ReadDAG_Task(path, "RM");
    dagTasks.AddDummyNode();
}
TEST(transform, v1)
{
    std::vector<std::string> paths;
    paths.push_back("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/dag-set-000-syntheticJobs.csv");
    paths.push_back("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/dag-set-001-syntheticJobs.csv");
    // paths.push_back("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/dag-set-002-syntheticJobs.csv");
    // paths.push_back("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/dag-set-003-syntheticJobs.csv");
    // paths.push_back("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/dag-set-004-syntheticJobs.csv");
    // auto dagTasksNumOpt = rt_num_opt::TransformTaskSetNumOpt2dagSched(paths);
    // dagSched::Taskset taskSetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(dagTasksNumOpt);
    // double rtaBase = dagSched::RTA_Nasri19(taskSetVerucchi, rt_num_opt::core_m_dag)[0];
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
