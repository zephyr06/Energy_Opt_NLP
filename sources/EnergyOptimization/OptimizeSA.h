#include <iostream>
#include <chrono>
#include <cmath>
#include <time.h>

#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_LL.h"

#include "../includeMoe/moe/moe.hpp"
namespace rt_num_opt
{
    template <class TaskSetType, class Schedul_Analysis>
    OptimizeResult OptimizeSchedulingSA(TaskSetType &tasks)
    {
        srand(0);
        int N = tasks.tasks_.size();
        TASK_NUMBER = N;
        vectorGlobalOpt.resize(N, 1);
        vectorGlobalOpt.setZero();
        // auto hyperPeriod = HyperPeriod(tasks);
        int lastTaskDoNotNeedOptimize = -1;
        VectorDynamic initialEstimate = GetParameterVD<int>(tasks, "executionTime");
        VectorDynamic periods = GetParameterVD<int>(tasks, "period");
        Schedul_Analysis r(tasks);
        VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();

        if (!r.CheckSchedulabilityDirect(responseTimeInitial))
            return {INT_MAX, INT_MAX,
                    initialEstimate, initialEstimate};
        gtsam::Symbol key('a', 0);
        auto model = gtsam::noiseModel::Isotropic::Sigma((N - lastTaskDoNotNeedOptimize - 1), noiseModelSigma);
        typename Energy_Opt<TaskSetType, Schedul_Analysis>::ComputationFactor factor(key, tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, model);

        moe::SimulatedAnnealing<double> moether(moe::SAParameters<double>()
                                                    .withTemperature(temperatureSA)
                                                    .withCoolingRate(coolingRateSA)
                                                    .withDimensions(N)
                                                    .withRange({initialEstimate.minCoeff(), periods.maxCoeff()}));

        moether.setFitnessFunction([&](auto startTimeVec) -> double
                                   {
                                       VectorDynamic startTimeVector = Vector2Eigen<double>(startTimeVec.genotype);
                                       VectorDynamic err;
                                       err = factor.evaluateError(startTimeVector);
                                       return err.norm() *-1; });

        auto start = std::chrono::high_resolution_clock::now();

        auto initialSA = Eigen2Vector<double>(initialEstimate);
        if (randomInitialize)
            moether.run(SA_iteration);
        else
            moether.runSA(SA_iteration, initialSA, randomInitialize, tasks.tasks_);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> diff = end - start;

        auto best_moe = moether.getBestMoe();

        if (debugMode == 1)
        {
            std::cout << "Initial estimation for SA is " << initialEstimate << std::endl;
            std::cout
                << "fitness: " << best_moe.fitness * -1 << "\n"
                << "time spent: " << diff.count() << " seconds" << std::endl;
        }

        return {factor.evaluateError(initialEstimate).norm(), best_moe.fitness * -1,
                initialEstimate, Vector2Eigen<double>(best_moe.genotype)};
    }
} // namespace rt_num_opt