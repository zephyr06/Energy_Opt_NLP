#include "gtsam/base/Value.h"
#include "sources/Utils/GlobalVariables.h" // EliminationRecord

namespace rt_num_opt
{
    namespace EnergyOptUtils
    {
        static gtsam::Values
        GenerateInitialFG(TaskSet tasks, EliminationRecord &eliminationRecord)
        {
            gtsam::Values initialEstimateFG;
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (eliminationRecord[i].type == EliminationType::Not)
                    initialEstimateFG.insert(GenerateKey(i, "executionTime"),
                                             GenerateVectorDynamic1D(tasks[i].executionTime));
            }
            return initialEstimateFG;
        }

        static gtsam::Values
        GenerateInitialFG(TaskSetNormal tasks, EliminationRecord &eliminationRecord)
        {
            return GenerateInitialFG(tasks.tasks_, eliminationRecord);
        }

        VectorDynamic ExtractResults(const gtsam::Values &result, const TaskSet &tasks)
        {
            VectorDynamic executionTimes = GetParameterVD<double>(tasks, "executionTime");
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (result.exists(GenerateKey(i, "executionTime")))
                {
                    executionTimes(i, 0) = result.at<VectorDynamic>(GenerateKey(i, "executionTime"))(0, 0);
                }
            }
            return executionTimes;
        }
        VectorDynamic ExtractResults(const gtsam::Values &result, const TaskSetNormal &tasks)
        {
            return ExtractResults(result, tasks.tasks_);
        }

        double RealObj(const TaskSet tasks)
        {
            return EstimateEnergyTaskSet(tasks).sum() / weightEnergy;
        }
    } // namespace EnergyOptUtils

} // namespace rt_num_opt