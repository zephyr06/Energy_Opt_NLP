#include <CppUnitLite/TestHarness.h>

#include "sources/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Task.h"

#include "dagSched/DAGTask.h"
#include "dagSched/Taskset.h"

TEST(read, v1)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v33.csv";
    rt_num_opt::DAG_Model dagTasks = rt_num_opt::ReadDAG_Task(path, "RM");
    long long hyperPeriod = rt_num_opt::HyperPeriod(dagTasks.tasks_);

    dagSched::Taskset taskset;
    size_t taskSetSize = 1;
    for (size_t i = 0; i < taskSetSize; i++)
    {
        dagSched::DAGTask taskDAGCurr;
        // read and set data
        taskDAGCurr.setPeriod(static_cast<float>(hyperPeriod));
        taskDAGCurr.setDeadline(static_cast<float>(hyperPeriod));
        rt_num_opt::TaskSet tasksMy &= dagTasks.GetNormalTaskSet();

        // add tasks
        for (size_t j = 0; j < tasksMy.size(); j++)
        {
            SubTask *v = new SubTask;
            v->id = tasksMy[i].id;
            v->c = tasksMy[j].executionTime;
        }

        // add edges
        EXPECT_LONGS_EQUAL(4, num_edges(dagTasks.graph_));

        // processing by Verucchi
        taskDAGCurr.computeWorstCaseWorkload();
        taskDAGCurr.computeVolume();
        taskDAGCurr.computeLength();
        taskDAGCurr.computeUtilization();
        taskDAGCurr.computeDensity();

        tasks.push_back(taskDAGCurr);
    }

    computeUtilization();
    computeHyperPeriod();
    computeMaxDensity();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
