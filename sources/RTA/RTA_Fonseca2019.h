
#include "dagSched/DAGTask.h"
#include "dagSched/Taskset.h"
#include "dagSched/tests.h"

#include "sources/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Task.h"

dagSched::Taskset TransformTaskSetNumOpt2dagSched(rt_num_opt::DAG_Model dagTasks)
{
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
            // std::cout << from_id << ", " << to_id << std::endl;

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

dagSched::Taskset TransformTaskSetNumOpt2dagSched(std::string path)
{

    rt_num_opt::DAG_Model dagTasks = rt_num_opt::ReadDAG_Task(path, "RM");
    return TransformTaskSetNumOpt2dagSched(dagTasks);
}