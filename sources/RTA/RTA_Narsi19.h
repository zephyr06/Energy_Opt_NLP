#include "sources/TaskModel/DAG_Narsi19.h"

namespace rt_num_opt
{
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

    dagSched::Taskset TransformTaskSetNumOpt2dagSched(DAG_Narsi19 dagTasksNumOpt)
    {
        dagTasksNumOpt.SyncTaskSet();
        return TransformTaskSetNumOpt2dagSched(dagTasksNumOpt.tasksVecNarsi_);
    }

    class RTA_Narsi19 : public RTA_BASE<DAG_Narsi19>
    {
    private:
        DAG_Narsi19 dagNarsi_;

    public:
        RTA_Narsi19(DAG_Narsi19 &dagNarsi) : RTA_BASE<DAG_Narsi19>(dagNarsi.tasks_)
        {
            dagNarsi_ = dagNarsi;
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