
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
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::
            UpdateTaskSetPeriodFromValues(dags_copy, result,
                                          maskForElimination);
        RTA_Nasri19 r(dags_copy);
        VectorDynamic rtaFromOpt = r.ResponseTimeOfTaskSet();
        cout << "After optimization, the rta vector is " << std::endl
             << rtaFromOpt << std::endl;
        cout << "The graph error is " << graph.error(result) << std::endl;
        cout << Color::def;
        cout << std::endl;
        cout << Color::blue;

        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::
            UpdateTaskSetPeriodFromValues(dags_copy, initialEstimateFG,
                                          maskForElimination);
        cout << "Before optimization, the actual obj is "
             << FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dags_copy,
                                                                    coeff)
             << std::endl;

        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::
            UpdateTaskSetPeriodFromValues(dags_copy, result,
                                          maskForElimination);
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

typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimerType;
bool ifTimeout(TimerType start_time) {
    auto curr_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time)
            .count() >= OverallTimeLimit) {
        std::cout << "\nTime out when running OptimizeOrder. Maximum time is "
                  << OverallTimeLimit << " seconds.\n\n";
        return true;
    }
    return false;
}

template <typename FactorGraphType, class TaskSetType>
void AssignBestInitialPriority(TaskSetType &taskSetType,
                               const VectorDynamic &coeff) {
    // RM
    taskSetType.InitializePriority();
    double min_error = FactorGraphType::RealObj(taskSetType, coeff);
    TaskSetType taskSetBest = taskSetType;

    // control gradient
    AssignPriorityBasedOnlyGradient(taskSetType, coeff);
    double err_cur = FactorGraphType::RealObj(taskSetType, coeff);
    if (err_cur < min_error) {
        min_error = err_cur;
        taskSetBest = taskSetType;
    }

    // control coeff
    taskSetType.AssignPriorityControl(coeff);
    err_cur = FactorGraphType::RealObj(taskSetType, coeff);
    if (err_cur < min_error) {
        min_error = err_cur;
        taskSetBest = taskSetType;
    }
    taskSetType = taskSetBest;
}

template <typename FactorGraphType, class TaskSetType>
void InitializePriorityAssignment(TaskSetType &taskSetType,
                                  const VectorDynamic &coeff,
                                  int enableReorderInput) {
    if (enableReorderInput == 0)
        return;
    else if (enableReorderInput == 1) {
        AssignBestInitialPriority<FactorGraphType, TaskSetType>(taskSetType,
                                                                coeff);
    } else if (enableReorderInput == 2)
        taskSetType.AssignPriorityRM();
    else if (enableReorderInput == 3) {
        AssignPriorityBasedOnlyGradient(taskSetType, coeff);
    } else
        CoutError("Unrecognized enableReorderInput argument!");
}
// only accepts DAG-related task set type, update taskSetType during
// optimization
template <typename FactorGraphType, class TaskSetType, class Schedul_Analysis>
static std::pair<VectorDynamic, double> OptimizeTaskSetIterative(
    TaskSetType &taskSetType, VectorDynamic &coeff,
    std::vector<bool> &maskForElimination) {
    auto run_time_track_start = std::chrono::high_resolution_clock::now();
    PeriodRoundQuantum =
        taskSetType.hyperPeriod / 10;  // for shorter run-time cost

    // double errPrev = 1e30;
    double errCurr = FactorGraphType::RealObj(taskSetType, coeff);
    if (errCurr >= 1e30)
        CoutError("The input DAG is not schedulable!");
    VectorDynamic periodResCurr, periodResPrev;
    std::vector<bool> maskForEliminationPrev = maskForElimination;
    double err_initial = errCurr;

    InitializePriorityAssignment<FactorGraphType, TaskSetType>(
        taskSetType, coeff, enableReorder);

    int loopCount = 0;
    // double disturbIte = eliminateTol;
    double weight_priority_assignment_during_iteration =
        weight_priority_assignment;
    double pa_change_threshold = Priority_assignment_adjustment_threshold;

    while (loopCount < MaxLoopControl && !(ifTimeout(run_time_track_start))) {
        // store prev result
        // errPrev = errCurr;
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
        BeginTimer("ChangePriorityAssignmentOrder");
        bool change_pa = false;
        if (enableReorder == 0)
            ;
        else if (enableReorder == 1) {
            std::vector<TaskPriority> pri_ass = ReorderWithGradient(
                taskSetType, coeff, weight_priority_assignment_during_iteration,
                pa_change_threshold);
            change_pa = UpdateAllTasksPriority(taskSetType, pri_ass);

            weight_priority_assignment_during_iteration =
                weight_priority_assignment_during_iteration /
                2;  // weight needs to converge to 0 so that the approximated
            // obj converges to the true obj
            pa_change_threshold +=
                Priority_assignment_threshold_incremental;  // in later
                                                            // iterations, less
                                                            // PA changes are
                                                            // needed
        } else if (enableReorder == 2) {
            taskSetType.AssignPriorityRM();
        } else if (enableReorder == 3) {
            AssignPriorityBasedOnlyGradient(taskSetType, coeff);
        } else
            CoutError("Unknown enblaeReorder option!");
        if (!CheckNasri19Schedulability(taskSetType))
            break;

        EndTimer("ChangePriorityAssignmentOrder");

        // adjust optimization settings
        if (!change_pa)
            FactorGraphType::FindEliminatedVariables(taskSetType,
                                                     maskForElimination);

        // RoundPeriod(tasks, maskForElimination, coeff);
        errCurr = FactorGraphType::RealObj(taskSetType, coeff);
        if (Equals(maskForElimination, maskForEliminationPrev) &&
            relativeErrorTolerance > relativeErrorToleranceMin) {
            relativeErrorTolerance = relativeErrorTolerance / 10;
        }

        loopCount++;
        if (debugMode) {
            using namespace std;
            cout << Color::green
                 << "Loop " + to_string_precision(loopCount, 4) + ": " +
                        std::to_string(errCurr)
                 << Color::def << std::endl;
            print(maskForElimination);
            cout << std::endl;
        }

        // termination conditions
        // if (errCurr >= errPrev * (1 - relativeErrorToleranceOuterLoop) &&
        //     change_pa == false && (!ContainFalse(maskForElimination)))
        //     break;
        if ((!ContainFalse(maskForElimination)))
            break;
    }
    // if (ContainFalse(maskForElimination)) {
    //     UpdateTaskSetPeriod(tasks, periodResPrev);
    //     RoundPeriod(tasks, maskForElimination, coeff);
    // } else {
    //     ;  // nothing else to do
    // }
    std::cout << "Overall loop: " << loopCount << "\n";
    RTA_Nasri19 r(taskSetType);
    if (r.CheckSchedulabilityLongTimeOut()) {
        return std::make_pair(
            GetParameterVD<double>(taskSetType.tasks_, "period"),
            FactorGraphType::RealObj(taskSetType, coeff) / err_initial);
    } else {
        CoutWarning("Return unschedulable result during control optimization!");
        if (enableReorder == 1)
            CoutError("Return unexpected unschedulable result!");
        return std::make_pair(GenerateVectorDynamic1D(0), 1);
    }
}
}  // namespace ControlOptimize
}  // namespace rt_num_opt