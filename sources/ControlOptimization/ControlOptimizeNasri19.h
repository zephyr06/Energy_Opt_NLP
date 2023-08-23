
#pragma once
#include <CppUnitLite/TestHarness.h>

#include <chrono>
#include <numeric>
#include <string>
#include <utility>

// #include "sources/ControlOptimization/CoeffFactor.h"
// #include "sources/ControlOptimization/FactorGraphForceManifold.h"
#include "sources/ControlOptimization/AdjustPriority.h"
#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
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
    const TaskSetType &taskSetType, VectorDynamic &coeff,
    std::vector<bool> &maskForElimination) {
    BeginTimer(__func__);
    gtsam::NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(
        taskSetType, maskForElimination, coeff);
    gtsam::Values initialEstimateFG =
        FactorGraphType::GenerateInitialFG(taskSetType, maskForElimination);
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

    VectorDynamic optComp;
    optComp = FactorGraphType::ExtractDAGPeriodVec(result, taskSetType);
    if (debugMode == 1) {
        using namespace std;
        cout << std::endl;
        cout << Color::blue;
        cout << "After optimization, the period vector is " << std::endl
             << optComp << std::endl;
        TaskSetType dags_copy = taskSetType;
        FactorGraphNasri<DAG_Nasri19,
                         RTA_Nasri19>::UpdateTaskSetPeriodFromValues(dags_copy,
                                                                     result);
        RTA_Nasri19 r(dags_copy);
        VectorDynamic rtaFromOpt = r.ResponseTimeOfTaskSet();
        cout << "After optimization, the rta vector is " << std::endl
             << rtaFromOpt << std::endl;
        cout << "The graph error is " << graph.error(result) << std::endl;
        cout << Color::def;
        cout << std::endl;
        cout << Color::blue;

        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::
            UpdateTaskSetPeriodFromValues(dags_copy, initialEstimateFG);
        cout << "Before optimization, the actual obj is "
             << FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dags_copy,
                                                                    coeff)
             << std::endl;

        FactorGraphNasri<DAG_Nasri19,
                         RTA_Nasri19>::UpdateTaskSetPeriodFromValues(dags_copy,
                                                                     result);
        cout << "After optimization, the actual obj is"
             << FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dags_copy,
                                                                    coeff)
             << std::endl;
        cout << Color::def;
    }

    EndTimer(__func__);
    return std::make_pair(optComp,
                          FactorGraphType::RealObj(taskSetType, coeff));
}

// void RoundPeriod(TaskSet &tasks, std::vector<bool> &maskForElimination,
//                  VectorDynamic &coeff) {
//     if (roundTypeInClamp == "none")
//         return;
//     else if (roundTypeInClamp == "rough") {
//         for (uint i = 0; i < maskForElimination.size(); i++) {
//             if (maskForElimination[i]) {
//                 tasks[i].period = ceil(tasks[i].period);
//             }
//         }
//     } else {
//         std::cout << "input error in ClampComputationTime: " <<
//         roundTypeInClamp
//                   << std::endl;
//         throw;
//     }
//     return;
// }

// only accepts DAG-related task set type, update taskSetType during
// optimization
template <typename FactorGraphType, class TaskSetType, class Schedul_Analysis>
static std::pair<VectorDynamic, double> OptimizeTaskSetIterative(
    TaskSetType &taskSetType, VectorDynamic &coeff,
    std::vector<bool> &maskForElimination) {
    VectorDynamic periodResCurr, periodResPrev;
    std::vector<bool> maskForEliminationPrev = maskForElimination;
    RTA_Nasri19 rr(taskSetType);
    if (!rr.CheckSchedulability())
        CoutError("The input DAG is not schedulable!");
    double errPrev = 1e30;
    double errCurr = FactorGraphType::RealObj(taskSetType, coeff);
    double err_initial = errCurr;
    int loopCount = 0;

    // double disturbIte = eliminateTol;
    while (errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop) &&
           ContainFalse(maskForElimination) && loopCount < MaxLoopControl) {
        // store prev result
        errPrev = errCurr;
        periodResPrev = GetPeriodVecNasri19(taskSetType);
        maskForEliminationPrev = maskForElimination;

        // perform optimization
        double err;
        std::tie(periodResCurr, err) =
            UnitOptimizationPeriod<FactorGraphType, TaskSetType>(
                taskSetType, coeff, maskForElimination);
        // UpdateTaskSetPeriod(taskSetType, periodResCurr);
        FactorGraphType::UpdateTaskSetWithPeriodVariable(taskSetType,
                                                         periodResCurr);

        // adjust tasks' priority based on RM
        if (enableReorder > 0) {
            // errCurr = FactorGraphType::RealObj(taskSetType, coeff);
            // TaskSet tasksTry = tasks;
            // VectorDynamic coeffTry = coeff;
            // Reorder(tasksTry, coeffTry);
            // double errCurrTry =
            //     FactorGraphType::RealObj(tasksTry, coeffTry, taskSetType);
            // if (errCurrTry < errCurr) {
            //     tasks = tasksTry;
            //     coeff = coeffTry;
            //     // whether_pa_changed = true;
            // }
            std::vector<TaskPriority> pri_ass = ReorderWithGradient(
                taskSetType, coeff, weight_priority_assignment);
            UpdateAllTasksPriority(taskSetType, pri_ass);
        }

        // adjust optimization settings
        loopCount++;
        FactorGraphType::FindEliminatedVariables(taskSetType,
                                                 maskForElimination);

        // RoundPeriod(tasks, maskForElimination, coeff);
        errCurr = FactorGraphType::RealObj(taskSetType, coeff);
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
    // if (ContainFalse(maskForElimination)) {
    //     UpdateTaskSetPeriod(tasks, periodResPrev);
    //     RoundPeriod(tasks, maskForElimination, coeff);
    // } else {
    //     ;  // nothing else to do
    // }

    RTA_Nasri19 r(taskSetType);
    if (r.CheckSchedulability()) {
        return std::make_pair(
            GetParameterVD<double>(taskSetType.tasks_, "period"),
            FactorGraphType::RealObj(taskSetType, coeff) / err_initial);
    } else {
        CoutError("Return unschedulable result during control optimization!");
        return std::make_pair(GenerateVectorDynamic1D(0), 1e9);
    }
}
}  // namespace ControlOptimize
}  // namespace rt_num_opt