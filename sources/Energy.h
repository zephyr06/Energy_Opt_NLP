#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "Tasks.h"
#include "Parameters.h"

typedef Eigen::Matrix<double, TASK_NUMBER, 1> ComputationTimeVector;
typedef Eigen::Matrix<double, TASK_NUMBER, TASK_NUMBER> JacobianOpt;
typedef Eigen::Matrix<double, TASK_NUMBER, 1> ErrElement;

const float EstimateEnergyTask(const Task &task, float frequency)
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
{
    return task.executionTime * pow(frequency, 2) * weightEnergy;
}

ErrElement EstimateEnergyTaskSet(const TaskSet &taskSet, const ComputationTimeVector &executionTimeVector)
{
    int N = taskSet.size();
    int hp = HyperPeriod(taskSet);
    ErrElement res;
    for (int i = 0; i < N; i++)
    {
        float tt = taskSet[i].executionTime / executionTimeVector(i, 0);
        tt = hp / taskSet[i].period *
             EstimateEnergyTask(taskSet[i], tt);
        res(i, 0) = tt;
    }
    return res;
}
