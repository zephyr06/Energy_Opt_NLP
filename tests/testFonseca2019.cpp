#include <CppUnitLite/TestHarness.h>

#include "dagSched/DAGTask.h"
// #include "DAG-scheduling_Verucchi/src/DAGTask/DAGTask.cpp"
#include "dagSched/Taskset.h"
#include "dagSched/tests.h"

#include "sources/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Task.h"

dagSched::Taskset TransformTaskSetNumOpt2dagSched(std::string path)
{

    rt_num_opt::DAG_Model dagTasks = rt_num_opt::ReadDAG_Task(path, "RM");
    dagTasks.AddDummyNode();

    long long hyperPeriod = rt_num_opt::HyperPeriod(dagTasks.tasks_);

    dagSched::Taskset taskset;
    size_t taskSetSize = 1;
    for (size_t i = 0; i < taskSetSize; i++)
    {
        std::vector<dagSched::SubTask *> given_V;
        dagSched::DAGTask taskDAGCurr;
        // read and set data
        taskDAGCurr.setPeriod(static_cast<float>(hyperPeriod));
        taskDAGCurr.setDeadline(static_cast<float>(hyperPeriod));
        rt_num_opt::TaskSet tasksMy = dagTasks.GetNormalTaskSet();

        // add tasks
        for (size_t j = 0; j < tasksMy.size(); j++)
        {
            dagSched::SubTask *v = new dagSched::SubTask;
            // v->id = tasksMy[i].id;
            v->id = j;
            v->c = tasksMy[j].executionTime;
            given_V.push_back(v);
        }

        // add edges

        auto vertex2index_ = boost::get(boost::vertex_name, dagTasks.graph_);
        std::pair<rt_num_opt::edge_iter, rt_num_opt::edge_iter> ep;
        rt_num_opt::edge_iter ei, ei_end;
        for (tie(ei, ei_end) = boost::edges(dagTasks.graph_); ei != ei_end; ++ei)
        {
            rt_num_opt::Vertex vv = boost::source(*ei, dagTasks.graph_);
            int from_id = vertex2index_[vv];
            rt_num_opt::Vertex vt = boost::target(*ei, dagTasks.graph_);
            int to_id = vertex2index_[vt];
            std::cout << from_id << ", " << to_id << std::endl;

            given_V[from_id]->succ.push_back(given_V[to_id]);
            given_V[to_id]->pred.push_back(given_V[from_id]);
        }

        taskDAGCurr.setVertices(given_V);

        // processing by Verucchi
        taskDAGCurr.transitiveReduction();
        taskDAGCurr.computeWorstCaseWorkload();
        taskDAGCurr.computeVolume();
        taskDAGCurr.computeLength();
        taskDAGCurr.computeUtilization();
        taskDAGCurr.computeDensity();

        taskset.tasks.push_back(taskDAGCurr);
    }

    taskset.computeUtilization();
    taskset.computeHyperPeriod();
    taskset.computeMaxDensity();
    return taskset;
}
// TEST(read, v1)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v33.csv";
//     auto tasksetVerucchi = TransformTaskSetNumOpt2dagSched(path);

//     EXPECT_DOUBLES_EQUAL(90 + 69 + 4, tasksetVerucchi.tasks[0].getLength(), 1e-3);
// }

// TEST(GP_FP_FTP_Fonseca2019, v1)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v34.csv";
//     auto tasksetVerucchi = TransformTaskSetNumOpt2dagSched(path);
//     int n_proc = 2;
//     std::cout << std::endl
//               << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Fonseca2019(tasksetVerucchi, n_proc) << std::endl;
// }

TEST(GP_FP_FTP_Fonseca2019, v2)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v25.csv";
    auto tasksetVerucchi = TransformTaskSetNumOpt2dagSched(path);
    int n_proc = 2;
    std::cout << std::endl
              << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Fonseca2019(tasksetVerucchi, n_proc) << std::endl;
}

TEST(GP_FP_FTP_Fonseca2019, v3)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n8_v1.csv";
    auto tasksetVerucchi = TransformTaskSetNumOpt2dagSched(path);
    int n_proc = 4;
    std::cout << std::endl
              << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Fonseca2019(tasksetVerucchi, n_proc) << std::endl;
    EXPECT_LONGS_EQUAL(15, dagSched::RTA_Fonseca2019(tasksetVerucchi, n_proc)[0]);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
