/**
 * @file EnergyOptimize.h
 * @brief This file performs energy optimization subject to Nasri9's DAG
 * analysis model
 * @version 0.1
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <chrono>
#include <numeric>
#include <string>
#include <utility>

#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/EnergyOptimization/EnergyFactor.h"
#include "sources/EnergyOptimization/RTARelatedFactor.h"
#include "sources/MatrixConvenient.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
using namespace std::chrono;
namespace rt_num_opt {
MatrixDynamic GetNonzeroRow(MatrixDynamic &m) {
    std::vector<int> nonZeroRowIndex;
    for (uint i = 0; i < m.rows(); i++) {
        for (uint j = 0; j < m.cols(); j++) {
            if (m(i, j) != 0) {
                nonZeroRowIndex.push_back(i);
                break;
            }
        }
    }
    int rows = nonZeroRowIndex.size();
    int cols = m.cols();
    MatrixDynamic res = GenerateMatrixDynamic(rows, cols);
    for (uint i = 0; i < nonZeroRowIndex.size(); i++) {
        res.block(i, 0, 1, cols) = m.block(nonZeroRowIndex[i], 0, 1, cols);
    }
    return res;
}

// this class is basically a namespace with template
template <class TaskSetType, class Schedul_Analysis>
class Energy_OptDAG {
   public:
    static gtsam::NonlinearFactorGraph BuildEnergyGraph(
        TaskSetType tasks, EliminationRecord &eliminationRecord) {
        gtsam::NonlinearFactorGraph graph;
        auto modelNormal =
            gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(
            1, noiseModelSigma / weightHardConstraint);

        for (uint i = 0; i < tasks.size(); i++) {
            if (eliminationRecord[i].type == EliminationType::Not) {
                graph.emplace_shared<EnergyFactor>(
                    GenerateKey(i, "executionTime"), tasks[i], i, modelNormal);
                // add executionTime min/max limits
                graph.emplace_shared<LargerThanFactor1D>(
                    GenerateKey(i, "executionTime"), tasks[i].executionTimeOrg,
                    modelPunishmentSoft1);

                graph.emplace_shared<SmallerThanFactor1D>(
                    GenerateKey(i, "executionTime"),
                    tasks[i].executionTimeOrg * MaxComputationTimeRestrict,
                    modelPunishmentSoft1);
            }
        }

        // RTA factor
        graph.add(GenerateRTARelatedFactor<TaskSetType, Schedul_Analysis>(
            tasks, eliminationRecord));
        return graph;
    }

    static std::pair<VectorDynamic, double> UnitOptimization(
        TaskSetType &tasks, EliminationRecord &eliminationRecord) {
        BeginTimer(__func__);
        gtsam::NonlinearFactorGraph graph =
            BuildEnergyGraph(tasks, eliminationRecord);
        gtsam::Values initialEstimateFG =
            EnergyOptUtils::GenerateInitialFG(tasks, eliminationRecord);
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
            gtsam::LevenbergMarquardtOptimizer optimizer(
                graph, initialEstimateFG, params);
            result = optimizer.optimize();
            // print some messages
            if (debugMode == 1) {
                std::cout << "*****************************************"
                          << std::endl;
                std::cout << "Inner iterations "
                          << optimizer.getInnerIterations() << std::endl;
                std::cout << "lambda " << optimizer.lambda() << std::endl;
            }
        }

        VectorDynamic optComp,
            rtaFromOpt;  // rtaFromOpt can only be used for 'cout'
        optComp = EnergyOptUtils::ExtractResults(result, tasks);
        UpdateTaskSetExecutionTime(tasks.tasks_, optComp);

        Schedul_Analysis r(tasks);
        rtaFromOpt = r.ResponseTimeOfTaskSet();
        if (debugMode == 1) {
            std::cout << std::endl;
            std::cout << Color::blue;
            std::cout << "After optimization, the executionTime vector is "
                      << std::endl
                      << optComp << std::endl;
            std::cout << "After optimization, the rta vector is " << std::endl
                      << rtaFromOpt << std::endl;
            std::cout << "The graph error is " << graph.error(result)
                      << std::endl;
            std::cout << Color::def;

            std::cout << Color::blue;
            VectorDynamic newExecutionTime =
                EnergyOptUtils::ExtractResults(initialEstimateFG, tasks);
            UpdateTaskSetExecutionTime(tasks.tasks_, newExecutionTime);
            std::cout << "Before optimization, the total error is "
                      << EnergyOptUtils::RealObj(tasks.tasks_) << std::endl;
            UpdateTaskSetExecutionTime(tasks.tasks_, optComp);
            std::cout << "After optimization, the total error is "
                      << EnergyOptUtils::RealObj(tasks.tasks_) << std::endl;
            std::cout << Color::def;
        }

        EndTimer(__func__);
        return std::make_pair(optComp, EnergyOptUtils::RealObj(tasks.tasks_));
    }

    static std::pair<bool, EliminationRecord> FindEliminateVariable(
        const TaskSetType &tasks, EliminationRecord &eliminationRecord) {
        TaskSetType tasksCurr = tasks;
        bool whetherEliminate = false;
        double disturb = eliminateTol;
        for (; whetherEliminate == false && disturb < disturb_max;
             disturb *= eliminateStep) {
            for (int i = 0; i < tasks.N; i++) {
                if (eliminationRecord[i].type != EliminationType::Not)
                    continue;
                else {
                    double direction =
                        JacobianInEnergyItem(tasksCurr.tasks_, i);
                    tasksCurr.tasks_[i].executionTime -=
                        direction / std::abs(direction) * disturb;
                    Schedul_Analysis r(tasksCurr);
                    if (enableMaxComputationTimeRestrict &&
                        tasksCurr.tasks_[i].executionTime >=
                            MaxComputationTimeRestrict *
                                tasksCurr.tasks_[i].executionTimeOrg) {
                        eliminationRecord.SetEliminated(i,
                                                        EliminationType::Bound);
                        whetherEliminate = true;
                    } else if (!r.CheckSchedulability()) {
                        eliminationRecord.SetEliminated(i,
                                                        EliminationType::RTA);
                        whetherEliminate = true;
                    }
                    tasksCurr.tasks_[i].executionTime +=
                        direction / std::abs(direction) * disturb;
                }
            }
            if (eliminationRecord.whetherAllEliminated()) break;
        }
        if (debugMode == 1) {
            eliminationRecord.Print();
            std::cout << "disturb to trigger elimination: " << disturb
                      << std::endl;
        }

        return std::make_pair(whetherEliminate, eliminationRecord);
    }

    static std::pair<VectorDynamic, double> OptimizeTaskSetIterative(
        TaskSetType &tasks) {
        EliminationRecord eliminationRecord;
        eliminationRecord.Initialize(tasks.size());

        VectorDynamic executionTimeResCurr, executionTimeResPrev;
        double errPrev = 1e30;
        double errCurr = EnergyOptUtils::RealObj(tasks.tasks_);
        double initialError = errCurr;
        int loopCount = 0;
        bool whether_new_eliminate = false;
        while (whether_new_eliminate ||
               (errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop))) {
            // store prev result
            errPrev = errCurr;
            executionTimeResPrev =
                GetParameterVD<double>(tasks, "executionTime");

            std::tie(executionTimeResCurr, errCurr) =
                UnitOptimization(tasks, eliminationRecord);

            std::tie(whether_new_eliminate, eliminationRecord) =
                FindEliminateVariable(tasks, eliminationRecord);

            loopCount++;
            if (loopCount > elimIte || eliminationRecord.whetherAllEliminated())
                break;
        }

        double postError = EnergyOptUtils::RealObj(tasks.tasks_);
        std::cout
            << "The number of outside loops in OptimizeTaskSetIterative is "
            << loopCount << std::endl;
        // std::cout << "Best optimal found: " << valueGlobalOpt << std::endl;
        std::cout << "After optimiazation found: " << postError << std::endl;
        if (valueGlobalOpt < postError) {
            UpdateTaskSetExecutionTime(tasks.tasks_, vectorGlobalOpt);
        }

        // verify feasibility
        Schedul_Analysis xx(tasks);
        if (xx.CheckSchedulability() == false) {
            CoutError("Unfeasible result found! Infeasible after optimization");
        }
        for (uint i = 0; i < tasks.size(); i++) {
            if (enableMaxComputationTimeRestrict &&
                tasks.tasks_[i].executionTime >
                    MaxComputationTimeRestrict *
                            tasks.tasks_[i].executionTimeOrg +
                        1e-3) {
                CoutWarning(
                    "Unfeasible result found! Bound constraint violated by " +
                    std::to_string(tasks.tasks_[i].executionTime -
                                   MaxComputationTimeRestrict *
                                       tasks.tasks_[i].executionTimeOrg));
                break;
            }
        }
        return std::make_pair(
            GetParameterVD<double>(tasks, "executionTime"),
            EnergyOptUtils::RealObj(tasks.tasks_) / initialError);
    }
};
}  // namespace rt_num_opt