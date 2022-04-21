#pragma once
#include "RTA_LL.h"
namespace rt_num_opt
{
    // MatrixDynamic A_Global;
    // MatrixDynamic P_Global;
    class RTA_WAP : public RTA_BASE<TaskSetNormal>
    {
    public:
        RTA_WAP() {}
        RTA_WAP(const TaskSet &tasksI)
        {
            TaskSetNormal tasksN(tasksI);
            tasks = tasksN;
        }
        RTA_WAP(const TaskSetNormal &tasks) : RTA_BASE(tasks) {}

        static std::string type()
        {
            return "WAP";
        }
        // ******************** Standard interfaces to use ********************************** //
        double RTA_Common_Warm(double beginTime, int index) override
        {
            if (A_Global.rows() && P_Global.rows())
            {
                double block = BlockingTime(tasks.tasks_, A_Global, P_Global, index);
                return RTA_blockWarm(beginTime, tasks.tasks_, A_Global, P_Global, index, block);
            }
            else
            {
                CoutError("A_Global, P_Global undefined!");
            }
            return -1;
        }
        double RTA_Common(int index)
        {
            double beginTime = tasks.tasks_.at(index).executionTime;
            return RTA_Common_Warm(beginTime, index);
        }

        // ******************** helper functions to use ********************************** //
        /**
     * @brief based on WAP model
     * 
     * @param tasks 
     * @param A 
     * @param P 
     * @param index 
     * @return double 
     */
        static double BlockingTime(const TaskSet &tasks, const MatrixDynamic &A, const MatrixDynamic &P, int index)
        {
            int N = tasks.size();
            double block = 0;
            for (int l = index + 1; l < N; l++)
            {
                if (A.coeff(index, l) == 0 && P.coeff(index, l) == 1)
                {
                    block = max(block, tasks[l].overhead);
                }
                else if (A.coeff(index, l) == 0 && P.coeff(index, l) == 0)
                {
                    block = max(block, tasks[l].executionTime - 1);
                }
                else
                    block = max(block, 0);
            }
            return block;
        }

        /**
     * @brief find equivalent hp tasks
     * 
     * @param tasks all the tasks are ordered via inverse priority
     * @param A 
     * @param P 
     * @param index 
     * @return TaskSet
     */
        static TaskSet EquivalentHpTasks(const TaskSet &tasks, const MatrixDynamic &A, const MatrixDynamic &P, int index)
        {
            // int N = tasks.size();
            TaskSet tasksHp;
            tasksHp.reserve(index);
            for (int i = 0; i < index; i++)
                tasksHp.push_back(tasks.at(i));
            for (int j = 0; j < index; j++)
            {
                double abortOverhead = 0;
                for (int k = j + 1; k < index + 1; k++)
                {
                    if (A.coeff(j, k) == 1)
                        abortOverhead = max(abortOverhead, tasks[k].executionTime - 1);
                }
                if (abortOverhead == 0 && P.coeff(j, index) == 1)
                    abortOverhead = tasks[index].overhead;
                tasksHp[j].executionTime += abortOverhead;
            }
            return tasksHp;
        }

        static double GetBusyPeriod(const TaskSet &tasks,
                                    const MatrixDynamic &A, const MatrixDynamic &P, int index, double block = -1)
        {
            if (block == -1)
                block = BlockingTime(tasks, A, P, index);

            TaskSet tasksHp = EquivalentHpTasks(tasks, A, P, index);

            tasksHp.push_back(tasks.at(index));
            Task task = tasks.at(index);
            task.executionTime = block + tasks.at(index).executionTime;
            RTA_LL r(tasks);
            double bp = r.ResponseTimeAnalysisWarm_util_nece(task.executionTime, task, tasksHp);
            if (bp == INT32_MAX)
                return HyperPeriod(tasks);
            return bp;
        }

        /**
     * @brief RTA given block time
     * 
     * @param tasks 
     * @param A 
     * @param P 
     * @param index 
     * @param block 
     * @return double 
     */
        static double RTA_block(const TaskSet &tasks, const MatrixDynamic &A, const MatrixDynamic &P, int index, double block)
        {
            return RTA_blockWarm(tasks.at(index).executionTime, tasks, A, P, index, block);
        }

        static double RTA_blockWarm(double beginTime, const TaskSet &tasks, const MatrixDynamic &A, const MatrixDynamic &P, int index, double block)
        {
            Task taskCurr = tasks.at(index);
            // taskCurr.executionTime += block;
            TaskSet tasksHp = EquivalentHpTasks(tasks, A, P, index);
            double busyPeriod = GetBusyPeriod(tasks, A, P, index, block);
            if (busyPeriod == INT32_MAX)
            {
                // ???
                // busyPeriod = HyperPeriod(tasks);
                return INT32_MAX;
            }
            double wcrt = 0;
            for (int i = 0; i <= ceil(busyPeriod / tasks[index].period); i++)
            {
                taskCurr.executionTime = (1 + i) * tasks.at(index).executionTime + block;
                RTA_LL r(tasks);
                double instance_rt = r.ResponseTimeAnalysisWarm_util_nece(beginTime, taskCurr, tasksHp) - i * tasks.at(index).period;
                wcrt = max(wcrt, instance_rt);
                if (wcrt > tasks.at(index).period)
                    return wcrt * 1000;
            }
            return wcrt;
        }
    };
} // namespace rt_num_opt