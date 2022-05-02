
#include "dagSched/DAGTask.h"
#include "dagSched/Taskset.h"
#include "dagSched/tests.h"

#include "sources/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Fonseca.h"
#include "sources/RTA/RTA_BASE.h"

namespace rt_num_opt
{

    dagSched::DAGTask TransformSingleTaskNumOpt2dagSched(rt_num_opt::DAG_Model dagTasks)
    {
        dagTasks.AddDummyNode();

        long long hyperPeriod = rt_num_opt::HyperPeriod(dagTasks.tasks_);

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

        return taskDAGCurr;
    }

    std::vector<rt_num_opt::DAG_Model> TransformTaskSetNumOpt2dagSched(std::vector<std::string> paths)
    {
        std::vector<rt_num_opt::DAG_Model> taskset;
        size_t taskSetSize = paths.size();
        for (size_t i = 0; i < taskSetSize; i++)
        {
            auto dagTasks = rt_num_opt::ReadDAG_Task(paths[i], "RM");
            // dagTasks.AddDummyNode();
            taskset.push_back(dagTasks);
        }
        return taskset;
    }

    dagSched::Taskset TransformTaskSetNumOpt2dagSched(std::vector<rt_num_opt::DAG_Model> dagTasksNumOpt)
    {
        dagSched::Taskset taskset;
        size_t taskSetSize = dagTasksNumOpt.size();
        for (size_t i = 0; i < taskSetSize; i++)
        {
            auto dagTasks = dagTasksNumOpt[i];
            dagSched::DAGTask taskDAGCurr = TransformSingleTaskNumOpt2dagSched(dagTasks);
            taskset.tasks.push_back(taskDAGCurr);
        }
        taskset.computeUtilization();
        taskset.computeHyperPeriod();
        taskset.computeMaxDensity();
        return taskset;
    }

    class RTA_Narsi19 : public RTA_BASE<DAG_Fonseca>
    {
    private:
        DAG_Model dagModel_;

    public:
        RTA_Narsi19(DAG_Fonseca &dagModelFonseca) : RTA_BASE<DAG_Fonseca>(dagModelFonseca)
        {
            dagModel_ = dagModelFonseca.GetDAG_Model();
        }
        RTA_Narsi19(DAG_Model &dagModel) : RTA_BASE<DAG_Fonseca>(DAG_Fonseca(dagModel))
        {
            dagModel_ = dagModel;
        }

        double RTA_Common_Warm(double beginTime, int index) override
        {
            if (index < dagModel_.N - 1)
            {
                // Fonseca19 can only return end-to-end latency for a single DAG
                // not appropriate, but should do the work in optimize.h
                return 0;
            }
            else
            {
                // dagSched::Taskset taskSetVerucchi = TransformTaskSetNumOpt2dagSched(dagModel_);
                // return dagSched::RTA_G_LP_FTP_Nasri2019_C(taskSetVerucchi, rt_num_opt::core_m_dag)[0];
                return 0;
            }
        }
    };
}