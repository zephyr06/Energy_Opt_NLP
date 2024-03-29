
#pragma once
#include <CppUnitLite/TestHarness.h>

#include <chrono>
#include <numeric>
#include <string>
#include <utility>

// #include "sources/ControlOptimization/CoeffFactor.h"
// #include "sources/ControlOptimization/FactorGraphForceManifold.h"
#include "sources/ControlOptimization/FactorGraphInManifold.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/InequalifyFactor.h"
// #include "Optimize.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
namespace rt_num_opt {
namespace ControlOptimize {
template <typename FactorGraphType, typename TaskSetType>
std::pair<VectorDynamic, double> UnitOptimizationPeriod(
    TaskSet &tasks, VectorDynamic &coeff, std::vector<bool> &maskForElimination,
    const TaskSetType &taskSetType) {
    BeginTimer(__func__);
    gtsam::NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(
        maskForElimination, tasks, coeff, taskSetType);
    gtsam::Values initialEstimateFG =
        FactorGraphType::GenerateInitialFG(tasks, maskForElimination);
    if (debugMode == 1) {
        std::cout << Color::green;
        auto sth = graph.linearize(initialEstimateFG)->jacobian();
        MatrixDynamic jacobianCurr = sth.first;
        std::cout << "Current Jacobian matrix:" << std::endl;
        std::cout << jacobianCurr << std::endl;
        std::cout << "Current b vector: " << std::endl;
        std::cout << sth.second << std::endl;
        std::cout << Color::def << std::endl;
    }

    gtsam::Values result;
    if (optimizerType == 1) {
        gtsam::DoglegParams params;
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        gtsam::DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    } else if (optimizerType == 2) {
        gtsam::LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        params.setVerbosityLM(verbosityLM);
        params.setDiagonalDamping(setDiagonalDamping);
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setLinearSolverType(linearOptimizerType);
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG,
                                                     params);
        result = optimizer.optimize();
    } else if (optimizerType == 3) {
        gtsam::GaussNewtonParams params;
        if (debugMode == 1)
            params.setVerbosity("DELTA");
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setLinearSolverType(linearOptimizerType);
        gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
        try {
            result = optimizer.optimize();
        } catch (...) {
            result = initialEstimateFG;
        }
    } else if (optimizerType == 4) {
        gtsam::NonlinearOptimizerParams params;
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setLinearSolverType(linearOptimizerType);
        if (debugMode == 1)
            params.setVerbosity("DELTA");
        params.setMaxIterations(maxIterationsOptimizer);
        gtsam::NonlinearConjugateGradientOptimizer optimizer(
            graph, initialEstimateFG, params);
        result = optimizer.optimize();
        try {
            result = optimizer.optimize();
        } catch (...) {
            result = initialEstimateFG;
        }
    }

    VectorDynamic optComp, rtaFromOpt;
    optComp = FactorGraphType::ExtractResults(result, tasks);
    UpdateTaskSetPeriod(tasks, optComp);
    rtaFromOpt = RTAVector(tasks);
    if (debugMode == 1) {
        using namespace std;
        cout << std::endl;
        cout << Color::blue;
        cout << "After optimization, the period vector is " << std::endl
             << optComp << std::endl;
        cout << "After optimization, the rta vector is " << std::endl
             << rtaFromOpt << std::endl;
        cout << "The graph error is " << graph.error(result) << std::endl;
        cout << Color::def;
        cout << std::endl;
        cout << Color::blue;
        UpdateTaskSetPeriod(
            tasks, FactorGraphType::ExtractResults(initialEstimateFG, tasks));
        // cout << "Before optimization, the total error is "
        //      << FactorGraphType::RealObj(tasks, coeff) << std::endl;
        UpdateTaskSetPeriod(tasks, optComp);
        // cout << "The objective function is "
        //      << FactorGraphType::RealObj(tasks, coeff) << std::endl;
        cout << Color::def;
    }

    UpdateTaskSetPeriod(tasks, optComp);
    EndTimer(__func__);
    return std::make_pair(optComp,
                          FactorGraphType::RealObj(tasks, coeff, taskSetType));
}

// template <typename FactorGraphType>
// std::pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(
//     TaskSet &tasks, VectorDynamic &coeff,
//     std::vector<bool> &maskForElimination) {
//     VectorDynamic periodRes;
//     double err;
//     for (double weight = weightSchedulabilityMin;
//          weight <= weightSchedulabilityMax;
//          weight *= weightSchedulabilityStep) {
//         weightSchedulability = weight;
//         std::tie(periodRes, err) = UnitOptimizationPeriod<FactorGraphType>(
//             tasks, coeff, maskForElimination);
//         VectorDynamic periodPrev = GetParameterVD<double>(tasks, "period");
//         UpdateTaskSetPeriod(tasks, periodRes);
//     }

//     return std::make_pair(periodRes, err);
// }

/**
 * @brief round period into int type, we assume the given task set is
 * schedulable!
 *
 * @param tasks
 * @param maskForElimination
 * @param coeff
 */
void RoundPeriod(TaskSet &tasks, std::vector<bool> &maskForElimination,
                 VectorDynamic &coeff) {
    if (roundTypeInClamp == "none")
        return;
    else if (roundTypeInClamp == "rough") {
        for (uint i = 0; i < maskForElimination.size(); i++) {
            if (maskForElimination[i]) {
                tasks[i].period = ceil(tasks[i].period);
            }
        }
    } else if (roundTypeInClamp == "fine") {
        int N = tasks.size();

        std::vector<int> wait_for_eliminate_index;
        for (uint i = 0; i < tasks.size(); i++) {
            if (maskForElimination[i] &&
                tasks[i].period != ceil(tasks[i].period))
                wait_for_eliminate_index.push_back(i);
        }
        if (!wait_for_eliminate_index.empty()) {
            VectorDynamic rtaBase = RTAVector(tasks);

            std::vector<std::pair<int, double>> objectiveVec;
            objectiveVec.reserve(wait_for_eliminate_index.size());
            for (uint i = 0; i < wait_for_eliminate_index.size(); i++) {
                objectiveVec.push_back(std::make_pair(
                    wait_for_eliminate_index[i],
                    coeff(wait_for_eliminate_index[i] * 2) * -1));
            }
            sort(objectiveVec.begin(), objectiveVec.end(), comparePair);
            int iterationNumber = 0;

            while (objectiveVec.size() > 0) {
                int currentIndex = objectiveVec[0].first;

                // try to round 'up', if success, keep the loop; otherwise,
                // eliminate it and high priority tasks can be speeded up, if
                // necessary, by binary search
                int left = int(tasks[currentIndex].period);
                // int left = rtaBase(currentIndex);
                int right = ceil(tasks[currentIndex].period);
                if (left > right) {
                    CoutError("left > right error in clamp!");
                }
                int rightOrg = right;
                bool schedulale_flag;
                while (left < right) {
                    int mid = (left + right) / 2;

                    tasks[currentIndex].period = mid;
                    RTA_LL r(tasks);
                    schedulale_flag =
                        r.CheckSchedulability(rtaBase, debugMode == 1);
                    if (not schedulale_flag) {
                        left = mid + 1;
                        tasks[currentIndex].period = rightOrg;
                    } else {
                        tasks[currentIndex].period = mid;
                        right = mid;
                    }
                }

                tasks[currentIndex].period = left;
                objectiveVec.erase(objectiveVec.begin() + 0);

                iterationNumber++;
                if (iterationNumber > N) {
                    CoutWarning("iterationNumber error in Clamp!");
                    break;
                }
            };
        }
    } else {
        std::cout << "input error in ClampComputationTime: " << roundTypeInClamp
                  << std::endl;
        throw;
    }
    return;
}

bool ContainFalse(std::vector<bool> &eliminationRecord) {
    for (uint i = 0; i < eliminationRecord.size(); i++) {
        if (eliminationRecord[i] == false) {
            return true;
        }
    }
    return false;
}

template <typename FactorGraphType, class TaskSetType, class Schedul_Analysis>
static std::pair<VectorDynamic, double> OptimizeTaskSetIterative(
    TaskSet &tasks, VectorDynamic &coeff, std::vector<bool> &maskForElimination,
    const TaskSetType &taskSetType) {
    VectorDynamic periodResCurr, periodResPrev;
    std::vector<bool> maskForEliminationPrev = maskForElimination;
    double errPrev = 1e30;
    double errCurr = FactorGraphType::RealObj(tasks, coeff, taskSetType);
    int loopCount = 0;
    if (enableReorder > -1) {
        errCurr = FactorGraphType::RealObj(tasks, coeff, taskSetType);
        TaskSet tasksTry = tasks;
        VectorDynamic coeffTry = coeff;
        Reorder(tasksTry, coeffTry);
        double errCurrTry =
            FactorGraphType::RealObj(tasksTry, coeffTry, taskSetType);
        if (errCurrTry < errCurr) {
            tasks = tasksTry;
            coeff = coeffTry;
        }
    }
    // double disturbIte = eliminateTol;
    while (errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop) &&
           ContainFalse(maskForElimination) && loopCount < MaxLoopControl) {
        // store prev result
        errPrev = errCurr;
        periodResPrev = GetParameterVD<double>(tasks, "period");
        maskForEliminationPrev = maskForElimination;

        // perform optimization
        double err;
        std::tie(periodResCurr, err) =
            UnitOptimizationPeriod<FactorGraphType, TaskSetType>(
                tasks, coeff, maskForElimination, taskSetType);
        UpdateTaskSetPeriod(tasks, periodResCurr);

        // adjust tasks' priority based on RM
        // bool whether_pa_changed = false;
        if (enableReorder > 0) {
            errCurr = FactorGraphType::RealObj(tasks, coeff, taskSetType);
            TaskSet tasksTry = tasks;
            VectorDynamic coeffTry = coeff;
            Reorder(tasksTry, coeffTry);
            double errCurrTry =
                FactorGraphType::RealObj(tasksTry, coeffTry, taskSetType);
            if (errCurrTry < errCurr) {
                tasks = tasksTry;
                coeff = coeffTry;
                // whether_pa_changed = true;
            }
        }

        // adjust optimization settings
        loopCount++;
        // if (!whether_pa_changed)
        FactorGraphType::FindEliminatedVariables(tasks, maskForElimination,
                                                 taskSetType);

        RoundPeriod(tasks, maskForElimination, coeff);
        errCurr = FactorGraphType::RealObj(tasks, coeff, taskSetType);
        if (Equals(maskForElimination, maskForEliminationPrev) &&
            relativeErrorTolerance > relativeErrorToleranceMin) {
            relativeErrorTolerance = relativeErrorTolerance / 10;
        }
        if (debugMode) {
            using namespace std;
            cout << Color::green
                 << "Loop " + to_string_precision(loopCount, 4) + ": " +
                        std::to_string(errCurr)
                 << Color::def << std::endl;
            print(maskForElimination);
            cout << std::endl;
        }
    }
    if (ContainFalse(maskForElimination)) {
        UpdateTaskSetPeriod(tasks, periodResPrev);
        RoundPeriod(tasks, maskForElimination, coeff);
    } else {
        ;  // nothing else to do
    }

    RTA_LL r(tasks);
    if (r.CheckSchedulability()) {
        return std::make_pair(
            GetParameterVD<double>(tasks, "period"),
            FactorGraphType::RealObj(tasks, coeff, taskSetType));
    } else {
        VectorDynamic periodVecOrg = GetParameterVD<double>(tasks, "periodOrg");
        UpdateTaskSetPeriod(tasks, periodVecOrg);
        return std::make_pair(
            periodVecOrg, FactorGraphType::RealObj(tasks, coeff, taskSetType));
    }
}
}  // namespace ControlOptimize
}  // namespace rt_num_opt