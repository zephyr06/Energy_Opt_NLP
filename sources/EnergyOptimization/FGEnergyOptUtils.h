#include "gtsam/base/Value.h"

namespace rt_num_opt
{
    namespace EnergyOptUtils
    {

        static gtsam::Values
        GenerateInitialFG(TaskSet tasks)
        {
            gtsam::Values initialEstimateFG;
            for (uint i = 0; i < tasks.size(); i++)
            {
                initialEstimateFG.insert(GenerateKey(i, "executionTime"),
                                         GenerateVectorDynamic1D(tasks[i].executionTime));
            }
            return initialEstimateFG;
        }

        static gtsam::Values
        GenerateInitialFG(TaskSetNormal tasks)
        {
            return GenerateInitialFG(tasks.tasks_);
        }

        VectorDynamic ExtractResults(const gtsam::Values &result, const TaskSet &tasks)
        {
            VectorDynamic executionTimes = GenerateVectorDynamic(result.size());
            for (uint i = 0; i < result.size(); i++)
            {
                if (result.exists(GenerateKey(i, "executionTime")))
                {
                    executionTimes(i, 0) = result.at<VectorDynamic>(GenerateKey(i, "executionTime"))(0, 0);
                }
                else
                {
                    CoutError("Key not found in ExtractResults!");
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