/**
 * @file EnergyOptimize.h
 * @brief This file performs energy optimization subject to Nasri9's DAG analysis model
 * @version 0.1
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>

#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include "sources/EnergyOptimization/EnergyFactor.h"
#include "sources/EnergyOptimization/LockFactor.h"
#include "sources/EnergyOptimization/RTARelatedFactor.h"
#include "sources/Utils/Parameters.h"
#include "sources/MatrixConvenient.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/FactorGraphUtils.h"
#include <chrono>
using namespace std::chrono;
namespace rt_num_opt
{
    MatrixDynamic GetNonzeroRow(MatrixDynamic &m)
    {
        std::vector<int> nonZeroRowIndex;
        for (uint i = 0; i < m.rows(); i++)
        {
            for (uint j = 0; j < m.cols(); j++)
            {
                if (m(i, j) != 0)
                {
                    nonZeroRowIndex.push_back(i);
                    break;
                }
            }
        }
        int rows = nonZeroRowIndex.size();
        int cols = m.cols();
        MatrixDynamic res = GenerateMatrixDynamic(rows, cols);
        for (uint i = 0; i < nonZeroRowIndex.size(); i++)
        {
            res.block(i, 0, 1, cols) = m.block(nonZeroRowIndex[i], 0, 1, cols);
        }
        return res;
    }

    // this class is basically a namespace with template
    template <class TaskSetType, class Schedul_Analysis>
    class Energy_OptDAG
    {
    public:
        // this factor graph doesn't consider warm-start
        static gtsam::NonlinearFactorGraph BuildEnergyGraph(TaskSetType tasks)
        {
            gtsam::NonlinearFactorGraph graph;
            auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
            auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
            // auto modelPunishmentHard = gtsam::noiseModel::Constrained::All(1);

            for (uint i = 0; i < tasks.size(); i++)
            {
                // energy factor
                graph.emplace_shared<EnergyFactor>(GenerateKey(i, "executionTime"), tasks[i], i, modelNormal);

                // add executionTime min/max limits
                graph.emplace_shared<LargerThanFactor1D>(GenerateKey(i, "executionTime"), tasks[i].executionTimeOrg, modelPunishmentSoft1);

                graph.emplace_shared<SmallerThanFactor1D>(GenerateKey(i, "executionTime"), tasks[i].executionTimeOrg * MaxComputationTimeRestrict, modelPunishmentSoft1);

                if (eliminationRecordGlobal[i].type == EliminationType::Bound)
                {
                    graph.add(GenerateLockFactor(tasks.tasks_, i));
                }
            }

            // RTA factor
            graph.add(GenerateRTARelatedFactor<TaskSetType, Schedul_Analysis>(tasks));
            return graph;
        }

        /**
         * @brief This function and the following function consider the optimization problem:
         * min  C^T x
         * s.b. Jx <= 0
         *
         * @param tasks
         * @return NonlinearFactorGraph
         */
        static gtsam::NonlinearFactorGraph BuildGraphForC(TaskSetNormal &tasks)
        {
            gtsam::NonlinearFactorGraph graph;
            auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);

            for (uint i = 0; i < tasks.size(); i++)
            {
                // energy factor
                graph.emplace_shared<EnergyFactor>(GenerateKey(i, "executionTime"), tasks[i], i, modelNormal);
            }
            return graph;
        }

        static gtsam::Values MergeValuesInElimination(gtsam::Values initial, gtsam::VectorValues &delta)
        {
            return initial.retract(delta);
        }

        static std::pair<VectorDynamic, double> UnitOptimization(TaskSetType &tasks)
        {
            BeginTimer(__func__);
            gtsam::NonlinearFactorGraph graph = BuildEnergyGraph(tasks);
            gtsam::NonlinearFactorGraph graphForC = BuildGraphForC(tasks);
            // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
            // initialEstimate << 68.000000, 321, 400, 131, 308;
            gtsam::Values initialEstimateFG = EnergyOptUtils::GenerateInitialFG(tasks);
            if (debugMode == 1)
            {
                std::cout << Color::green;
                // std::lock_guard<std::mutex> lock(mtx);
                auto sth = graph.linearize(initialEstimateFG)->jacobian();
                MatrixDynamic jacobianCurr = sth.first;
                std::cout << "Current Jacobian matrix:" << std::endl;
                std::cout << jacobianCurr << std::endl;
                std::cout << "Current b vector: " << std::endl;
                std::cout << sth.second << std::endl;
                std::cout << Color::def << std::endl;
            }

            gtsam::Values result;
            if (optimizerType == 1)
            {
                gtsam::DoglegParams params;
                // if (debugMode == 1)
                //     params.setVerbosityDL("VERBOSE");
                params.setDeltaInitial(deltaInitialDogleg);
                params.setRelativeErrorTol(relativeErrorTolerance);
                gtsam::DoglegOptimizer optimizer(graph, initialEstimateFG, params);
                result = optimizer.optimize();
            }
            else if (optimizerType == 2)
            {
                gtsam::LevenbergMarquardtParams params;
                params.setlambdaInitial(initialLambda);
                params.setVerbosityLM(verbosityLM);
                params.setDiagonalDamping(setDiagonalDamping);
                params.setlambdaLowerBound(lowerLambda);
                params.setlambdaUpperBound(upperLambda);
                params.setRelativeErrorTol(relativeErrorTolerance);
                params.setLinearSolverType(linearOptimizerType);
                gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
                result = optimizer.optimize();
                // print some messages
                if (debugMode == 1)
                {
                    std::cout << "*****************************************" << std::endl;
                    std::cout << "Inner iterations " << optimizer.getInnerIterations() << std::endl;
                    std::cout << "lambda " << optimizer.lambda() << std::endl;
                }
            }

            if (debugMode == 1)
            {
                eliminationRecordGlobal.Print();
                eliminationRecordGlobal.PrintViolatedFactor();
            }

            std::cout << "Analyze descent direction:--------------------------" << std::endl;
            MatrixDynamic cDDMatrix = graphForC.linearize(result)->jacobian().first;
            VectorDynamic cDD = cDDMatrix.diagonal();
            std::cout << cDD << std::endl;
            std::cout << std::endl;
            // int a = 1;

            // auto start = high_resolution_clock::now();
            // auto sth = graph.error(initialEstimateFG);
            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(stop - start);
            // cout << "Evaluate error:" << duration.count() << endl;
            // start = high_resolution_clock::now();
            // auto sth2 = graph.linearize(initialEstimateFG);
            // stop = high_resolution_clock::now();
            // duration = duration_cast<microseconds>(stop - start);
            // cout << "linearize:" << duration.count() << endl;

            // cout << Color::green;
            // // std::lock_guard<std::mutex> lock(mtx);
            // auto sth3 = graph.linearize(initialEstimateFG)->jacobian();
            // MatrixDynamic jacobianCurr = sth3.first;
            // std::cout << "Current Jacobian matrix:" << endl;
            // std::cout << jacobianCurr << endl;
            // std::cout << "Current b vector: " << endl;
            // std::cout << sth3.second << endl;
            // cout << Color::def << endl;

            VectorDynamic optComp, rtaFromOpt; // rtaFromOpt can only be used for 'cout'
            optComp = EnergyOptUtils::ExtractResults(result, tasks);
            UpdateTaskSetExecutionTime(tasks.tasks_, optComp);
            // rtaFromOpt = RTAVector(tasks);
            Schedul_Analysis r(tasks);
            rtaFromOpt = r.ResponseTimeOfTaskSet();
            if (debugMode == 1)
            {
                std::cout << std::endl;
                std::cout << Color::blue;
                std::cout << "After optimization, the executionTime vector is " << std::endl
                          << optComp << std::endl;
                std::cout << "After optimization, the rta vector is " << std::endl
                          << rtaFromOpt << std::endl;
                std::cout << "The graph error is " << graph.error(result) << std::endl;
                std::cout << Color::def;

                std::cout << Color::blue;
                VectorDynamic newExecutionTime = EnergyOptUtils::ExtractResults(initialEstimateFG, tasks);
                UpdateTaskSetExecutionTime(tasks.tasks_, newExecutionTime);
                std::cout << "Before optimization, the total error is " << EnergyOptUtils::RealObj(tasks.tasks_) << std::endl;
                UpdateTaskSetExecutionTime(tasks.tasks_, optComp);
                std::cout << "After optimization, the total error is " << EnergyOptUtils::RealObj(tasks.tasks_) << std::endl;
                std::cout << Color::def;
            }

            // UpdateTaskSetExecutionTime(tasks, optComp);
            EndTimer(__func__);
            return std::make_pair(optComp, EnergyOptUtils::RealObj(tasks.tasks_));
        }

        static std::pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(TaskSetType &tasks)
        {
            Schedul_Analysis rr(tasks);
            if (!rr.CheckSchedulability(debugMode == 1))
            {
                CoutWarning("The task set is not schedulable!");
                return std::make_pair(GetParameterVD<double>(tasks, "executionTime"), 1e30);
            }

            VectorDynamic executionTimeRes;
            double err;
            std::tie(executionTimeRes, err) = UnitOptimization(tasks);
            UpdateTaskSetExecutionTime(tasks.tasks_, executionTimeRes);

            if (debugMode == 1)
            {
                VectorDynamic periodPrev = GetParameterVD<double>(tasks, "executionTime");
                Schedul_Analysis r(tasks);
                if (!r.CheckSchedulability(1 == debugMode))
                {
                    UpdateTaskSetExecutionTime(tasks.tasks_, periodPrev);
                    std::lock_guard<std::mutex> lock(mtx);
                    std::cout << Color::blue << "After one iterate on updating weight parameter,\
             the execution time become unschedulable and are"
                              << std::endl
                              << executionTimeRes << std::endl;
                    std::cout << Color::def;
                    return std::make_pair(periodPrev, err);
                }
                else
                {

                    std::lock_guard<std::mutex> lock(mtx);
                    std::cout << Color::blue << "After one iterate on updating weight parameter,\
             the execution time remain schedulable and are"
                              << std::endl
                              << executionTimeRes << std::endl;
                    std::cout << Color::def;
                }
            }

            return std::make_pair(executionTimeRes, err);
        }

        static bool FindEliminationRecordDiff(EliminationRecord &eliminationRecordPrev,
                                              EliminationRecord &eliminationRecordGlobal)
        {
            for (uint i = 0; i < eliminationRecordGlobal.record.size(); i++)
            {
                if (eliminationRecordPrev[i].type == EliminationType::Not && eliminationRecordGlobal[i].type != EliminationType::Not)
                {
                    return true;
                }
            }
            return false;
        }

        static void FindEliminateVariableFromRecordGlobal(const TaskSetType &tasks)
        {
            EliminationRecord eliminationRecordPrev = eliminationRecordGlobal;
            if (debugMode == 1)
            {
                eliminationRecordGlobal.Print();
            }

            gtsam::NonlinearFactorGraph graph = BuildEnergyGraph(tasks);
            gtsam::Values initialEstimateFG = EnergyOptUtils::GenerateInitialFG(tasks);
            gtsam::Values result;
            gtsam::LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            params.setVerbosityLM(verbosityLM);
            params.setlambdaLowerBound(lowerLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setLinearSolverType(linearOptimizerType);
            params.setlambdaUpperBound(upperLambda);
            if (debugMode == 1)
            {
                std::cout << Color::green;
                // std::lock_guard<std::mutex> lock(mtx);
                auto sth = graph.linearize(initialEstimateFG)->jacobian();
                MatrixDynamic jacobianCurr = sth.first;
                std::cout << "Current Jacobian matrix:" << std::endl;
                std::cout << jacobianCurr << std::endl;
                std::cout << "Current b vector: " << std::endl;
                std::cout << sth.second << std::endl;
                std::cout << Color::def << std::endl;
            }

            double lambdaCurr = upperLambda;

            while (lambdaCurr > lowerLambda)
            {
                params.setlambdaInitial(lambdaCurr / 10);
                params.setlambdaLowerBound(lambdaCurr / 10);
                params.setlambdaUpperBound(lambdaCurr);

                gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
                // result = optimizer.optimize();
                // result.print();
                // cout << endl;
                // optimizer.iterate();
                // Values result_new = optimizer.values();
                // result_new.print();
                // VectorDynamic aaa = EnergyOptUtils::ExtractResults(result_new, tasks);
                // gtsam::VectorValues delta = optimizer.getDelta(params);
                // if (debugMode == 1)
                // {
                //     delta.print();
                //     std::cout << std::endl;
                // }
                // double useless = graph.error(MergeValuesInElimination(initialEstimateFG, delta));
                if (FindEliminationRecordDiff(eliminationRecordPrev, eliminationRecordGlobal))
                {
                    break;
                }
                lambdaCurr /= 10.0;
            }
            if (debugMode == 1)
            {
                if (FindEliminationRecordDiff(eliminationRecordPrev, eliminationRecordGlobal))
                {
                    std::cout << "Find a new elimination" << std::endl;
                }
                else
                {
                    std::cout << Color::red << "No new elimination found, algorithm ends!" << std::endl
                              << Color::def;
                }
                eliminationRecordGlobal.Print();
            }
        }
        // TODO: limit the number of outer loops

        static std::pair<VectorDynamic, double> OptimizeTaskSetIterative(TaskSetType &tasks)
        {
            eliminationRecordGlobal.Initialize(tasks.size());
            InitializeGlobalVector(tasks.size());

            VectorDynamic executionTimeResCurr, executionTimeResPrev;
            EliminationRecord eliminationRecordPrev = eliminationRecordGlobal;
            double errPrev = 1e30;
            double errCurr = EnergyOptUtils::RealObj(tasks.tasks_);
            int loopCount = 0;
            // double disturbIte = eliminateTol;
            bool whether_new_eliminate = false;
            while (whether_new_eliminate || (loopCount < elimIte && errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop))) // &&
            {

                // store prev result
                errPrev = errCurr;
                executionTimeResPrev = GetParameterVD<double>(tasks, "executionTime");
                // cout << "Previous execution time vector: " << endl
                //      << executionTimeResPrev << endl;
                eliminationRecordPrev = eliminationRecordGlobal;

                // perform optimization
                double err;
                std::tie(executionTimeResCurr, err) = OptimizeTaskSetIterativeWeight(tasks);

                // adjust optimization settings
                loopCount++;

                // int eliminateIteCount = 0;
                // std::vector<bool> maskForEliminationCopy = maskForElimination;
                // while (EqualVector(maskForEliminationCopy, maskForElimination) && eliminateIteCount < adjustEliminateMaxIte)
                // {
                //     FindEliminatedVariables(tasks, maskForEliminationCopy, disturbIte);
                //     disturbIte *= eliminateStep;
                //     eliminateIteCount++;
                // }
                // maskForElimination = maskForEliminationCopy;

                // erase changes of eliminationRecord made during inner loops
                eliminationRecordGlobal = eliminationRecordPrev;

                FindEliminateVariableFromRecordGlobal(tasks);
                whether_new_eliminate = FindEliminationRecordDiff(eliminationRecordPrev, eliminationRecordGlobal);
                if (!ContainFalse(eliminationRecordGlobal))
                {
                    break;
                }
                // disturbIte = FindEliminatedVariables(tasks, whether_new_eliminate, disturbIte);

                errCurr = EnergyOptUtils::RealObj(tasks.tasks_);
                if (!whether_new_eliminate && relativeErrorTolerance > relativeErrorToleranceMin)
                {
                    relativeErrorTolerance = relativeErrorTolerance / 10;
                }
                // if (debugMode)
                // {
                // cout << Color::green << "Loop " + to_string_precision(loopCount, 4) + ": " + to_string(errCurr) << Color::def << endl;
                //     print(maskForElimination);
                //     cout << endl;
                // }
            }
            // if (ContainFalse(maskForElimination))
            // {
            //     UpdateTaskSetExecutionTime(tasks, executionTimeResPrev);
            // }
            // else
            // {
            //     ; //nothing else to do
            // }

            double postError = EnergyOptUtils::RealObj(tasks.tasks_);
            std::cout << "The number of outside loops in OptimizeTaskSetIterative is " << loopCount << std::endl;
            std::cout << "Best optimal found: " << valueGlobalOpt << std::endl;
            std::cout << "After optimiazation found: " << postError << std::endl;
            if (valueGlobalOpt < postError)
            {
                UpdateTaskSetExecutionTime(tasks.tasks_, vectorGlobalOpt);
            }

            // verify feasibility
            Schedul_Analysis xx(tasks);
            if (xx.CheckSchedulability() == false)
            {
                CoutError("Unfeasible result found! Infeasible after optimization");
            }
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (enableMaxComputationTimeRestrict && tasks.tasks_[i].executionTime > MaxComputationTimeRestrict * tasks.tasks_[i].executionTimeOrg + 1e-3)
                {
                    CoutWarning("Unfeasible result found! Bound constraint violated by " + std::to_string(tasks.tasks_[i].executionTime - MaxComputationTimeRestrict * tasks.tasks_[i].executionTimeOrg));
                    break;
                }
            }
            return std::make_pair(GetParameterVD<double>(tasks, "executionTime"), EnergyOptUtils::RealObj(tasks.tasks_));
        }
    };
} // namespace rt_num_opt