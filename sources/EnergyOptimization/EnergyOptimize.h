
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/MatirxConvenient.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/InequalifyFactor.h"

#include "sources/Utils/FactorGraphUtils.h"
#include <chrono>
using namespace std::chrono;
namespace rt_num_opt
{
    template <class TaskSetType, class Schedul_Analysis>
    class Energy_OptDAG
    {
    public:
        static double RealObj(const TaskSet tasks)
        {
            return EstimateEnergyTaskSet(tasks).sum() / weightEnergy;
        }
        gtsam::Values MergeValuesInElimination(gtsam::Values initial, gtsam::VectorValues &delta)
        {
            return initial.retract(delta);
        }
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

        std::pair<VectorDynamic, double> UnitOptimization(TaskSetType &tasks)
        {
            BeginTimer(__func__);

            gtsam::NonlinearFactorGraph graph = BuildControlGraph(tasks);

            gtsam::NonlinearFactorGraph graphForC = FactorGraphEnergyLL::BuildGraphForC(tasks);
            gtsam::NonlinearFactorGraph graphForJ = FactorGraphEnergyLL::BuildGraphForJ(tasks);
            if (debugMode == 1)
            {
                graph.print();
            }
            // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
            // initialEstimate << 68.000000, 321, 400, 131, 308;
            gtsam::Values initialEstimateFG = GenerateInitialFG(tasks);
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
            int exactJacobianRef = exactJacobian;
            exactJacobian = 1;
            auto jPair = graphForJ.linearize(result)->jacobian();
            MatrixDynamic jDDRaw = jPair.first;
            std::cout << jDDRaw << std::endl
                      << std::endl;
            MatrixDynamic jDD = GetNonzeroRow(jDDRaw);
            std::cout << jDD << std::endl;
            VectorDynamic jError = jPair.second;
            exactJacobian = exactJacobianRef;
            std::cout << jDD << std::endl;
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
            optComp = ExtractResults(result, tasks);
            UpdateTaskSetExecutionTime(tasks, optComp);
            rtaFromOpt = RTALLVector(tasks);
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
                VectorDynamic newExecutionTime = FactorGraphEnergyLL::ExtractResults(initialEstimateFG, tasks);
                UpdateTaskSetExecutionTime(tasks, newExecutionTime);
                std::cout << "Before optimization, the total error is " << RealObj(tasks) << std::endl;
                UpdateTaskSetExecutionTime(tasks, optComp);
                std::cout << "After optimization, the total error is " << RealObj(tasks) << std::endl;
                std::cout << Color::def;
            }

            // UpdateTaskSetExecutionTime(tasks, optComp);
            EndTimer(__func__);
            return std::make_pair(optComp, RealObj(tasks));
        }

        std::pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(TaskSetType &tasks)
        {
            Schedul_Analysis rr(tasks);
            if (!rr.CheckSchedulability(debugMode == 1))
            {
                CoutWarning("The task set is not schedulable!");
                return std::make_pair(GetParameterVD<double>(tasks, "executionTime"), 1e30);
            }
            VectorDynamic executionTimeRes;
            double err;

            weightSchedulability = weight;
            std::tie(executionTimeRes, err) = UnitOptimization(tasks);
            VectorDynamic periodPrev = GetParameterVD<double>(tasks, "executionTime");
            UpdateTaskSetExecutionTime(tasks, executionTimeRes);
            Schedul_Analysis r(tasks);
            if (!r.CheckSchedulability(1 == debugMode))
            {
                UpdateTaskSetExecutionTime(tasks, periodPrev);
                // if (debugMode == 1)
                // {
                std::lock_guard<std::mutex> lock(mtx);
                std::cout << Color::blue << "After one iterate on updating weight parameter,\
             the execution time become unschedulable and are"
                          << std::endl
                          << executionTimeRes << std::endl;
                std::cout << Color::def;
                // }

                return std::make_pair(periodPrev, err);
            }
            else
            {
                if (debugMode == 1)
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

        /**
         * @brief round period into int type, we assume the given task set is schedulable!
         *
         * @param tasks
         * @param coeff
         */
        void RoundExecutionTime(TaskSetType &tasks)
        {
            if (roundTypeInClamp == "none")
                return;
            else if (roundTypeInClamp == "rough")
            {
                for (uint i = 0; i < tasks.size(); i++)
                {
                    if (abs(tasks[i].executionTime - int(tasks[i].executionTime)) < 0.01 || int(tasks[i].executionTime) == 0)
                    {
                        tasks[i].executionTime = int(round(tasks[i].executionTime));
                    }
                    else
                    {
                        tasks[i].executionTime = int(tasks[i].executionTime);
                    }
                }
            }
            else
            {
                std::cout << "input error in ClampComputationTime: " << roundTypeInClamp << std::endl;
                throw;
            }
            return;
        }
        bool FindEliminationRecordDiff(EliminationRecord &eliminationRecordPrev,
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

        void FindEliminateVariableFromRecordGlobal(const TaskSetType &tasks)
        {
            EliminationRecord eliminationRecordPrev = eliminationRecordGlobal;
            if (debugMode == 1)
            {
                eliminationRecordGlobal.Print();
            }

            gtsam::NonlinearFactorGraph graph = BuildControlGraph(tasks);
            gtsam::Values initialEstimateFG = GenerateInitialFG(tasks);
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
                // VectorDynamic aaa = ExtractResults(result_new, tasks);
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

        std::pair<VectorDynamic, double> OptimizeTaskSetIterative(TaskSetType &tasks)
        {
            eliminationRecordGlobal.Initialize(tasks.size());
            InitializeGlobalVector(tasks.size());

            VectorDynamic executionTimeResCurr, executionTimeResPrev;
            EliminationRecord eliminationRecordPrev = eliminationRecordGlobal;
            double errPrev = 1e30;
            double errCurr = RealObj(tasks);
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

                RoundExecutionTime(tasks);
                errCurr = RealObj(tasks);
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
            //     RoundExecutionTime(tasks, maskForElimination);
            // }
            // else
            // {
            //     ; //nothing else to do
            // }

            double postError = RealObj(tasks);
            std::cout << "The number of outside loops in OptimizeTaskSetIterative is " << loopCount << std::endl;
            std::cout << "Best optimal found: " << valueGlobalOpt << std::endl;
            std::cout << "After optimiazation found: " << postError << std::endl;
            if (valueGlobalOpt < postError)
            {
                UpdateTaskSetExecutionTime(tasks, vectorGlobalOpt);
            }

            // verify feasibility
            Schedul_Analysis xx(tasks);
            if (xx.CheckSchedulability() == false)
            {
                CoutError("Unfeasible result found! Infeasible after optimization");
            }
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (enableMaxComputationTimeRestrict && tasks[i].executionTime > MaxComputationTimeRestrict * tasks[i].executionTimeOrg + 1e-3)
                {
                    CoutWarning("Unfeasible result found! Bound constraint violated by " + std::to_string(tasks[i].executionTime - MaxComputationTimeRestrict * tasks[i].executionTimeOrg));
                    break;
                }
            }
            return std::make_pair(GetParameterVD<double>(tasks, "executionTime"), RealObj(tasks));
        }
    };
} // namespace rt_num_opt