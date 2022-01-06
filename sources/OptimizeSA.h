#include <iostream>
#include <chrono>
#include <cmath>
#include <time.h>
#include "Optimize.h"
#include "RTA_LL.h"
#include "RTA_WAP.h"
#include "../includeMoe/moe/moe.hpp"
template <class Schedul_Analysis>
OptimizeResult OptimizeSchedulingSA(TaskSet &tasks)
{
    srand(0);
    int N = tasks.size();
    TASK_NUMBER = tasks.size();
    vectorGlobalOpt.resize(N, 1);
    vectorGlobalOpt.setZero();
    // auto hyperPeriod = HyperPeriod(tasks);
    int lastTaskDoNotNeedOptimize = -1;
    VectorDynamic initialEstimate = GetParameterVD<int>(tasks, "executionTime");
    VectorDynamic periods = GetParameterVD<int>(tasks, "period");
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    if (!CheckSchedulabilityDirect(tasks, responseTimeInitial))
        return {INT_MAX, INT_MAX,
                initialEstimate, initialEstimate};
    Symbol key('a', 0);
    auto model = noiseModel::Isotropic::Sigma((N - lastTaskDoNotNeedOptimize - 1), noiseModelSigma);
    Energy_Opt<RTA_LL>::ComputationFactor factor(key, tasks, lastTaskDoNotNeedOptimize,
                                                 responseTimeInitial, model);

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
                                   return err.norm() * -1;
                               });

    auto start = std::chrono::high_resolution_clock::now();

    auto initialSA = Eigen2Vector<double>(initialEstimate);
    if (randomInitialize)
        moether.run(SA_iteration);
    else
        moether.runSA(SA_iteration, initialSA, randomInitialize, tasks);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = end - start;

    auto best_moe = moether.getBestMoe();

    if (debugMode == 1)
    {
        cout << "Initial estimation for SA is " << initialEstimate << endl;
        std::cout
            << "fitness: " << best_moe.fitness * -1 << "\n"
            << "time spent: " << diff.count() << " seconds" << std::endl;
    }

    return {factor.evaluateError(initialEstimate).norm(), best_moe.fitness * -1,
            initialEstimate, Vector2Eigen<double>(best_moe.genotype)};
}
