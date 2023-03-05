/**
 * @file Optimize.h
 * @author Sen Wang
 * @brief This file mainly performs Energy optimization subject to simple RTA
 * model; it can optimize with Nasri19, though, but the elimination part is not
 * completed.
 * @version 0.1
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <math.h>

#include <Eigen/Dense>
#include <chrono>

#include "sources/EnergyOptimization/Energy.h"
#include "sources/EnergyOptimization/FrequencyModel.h"
#include "sources/MatrixConvenient.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/TaskModel/Tasks.h"
#include "sources/Utils/GlobalVariables.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/utils.h"
// #include "sources/EnergyOptimization/EnergyOptimize.h"
namespace rt_num_opt {

#define numberOfTasksNeedOptimize (N - lastTaskDoNotNeedOptimize - 1)

struct OptimizeResult {
    double initialError;
    double optimizeError;
    VectorDynamic initialVariable;
    VectorDynamic optimizeVariable;
    OptimizeResult() : initialError(-1), optimizeError(-1) { ; }
    OptimizeResult(double ie, double oe, VectorDynamic iv, VectorDynamic ov)
        : initialError(ie),
          optimizeError(oe),
          initialVariable(iv),
          optimizeVariable(ov) {}
};

bool CheckExecutionTimeBound(VectorDynamic &exec, TaskSet &tasksOrg) {
    if (not enableMaxComputationTimeRestrict) return true;
    for (uint i = 0; i < tasksOrg.size(); i++) {
        if (exec(i, 0) > tasksOrg[i].executionTime * 2) return false;
    }
    return true;
}

template <class TaskSetType, class Schedul_Analysis>
class Energy_Opt {
   public:
    class ComputationFactor : public gtsam::NoiseModelFactor1<VectorDynamic> {
       public:
        TaskSetType tasks_;
        int lastTaskDoNotNeedOptimize;
        VectorDynamic responseTimeInitial;
        int N;

        ComputationFactor(gtsam::Key key, TaskSetType &tasks,
                          int lastTaskDoNotNeedOptimize,
                          VectorDynamic responseTimeInitial,
                          gtsam::SharedNoiseModel model)
            : gtsam::NoiseModelFactor1<VectorDynamic>(model, key),
              tasks_(tasks),
              lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
              responseTimeInitial(responseTimeInitial) {
            N = tasks_.tasks_.size();
        }

        void UpdateGlobalVector(VectorDynamic &responseTimeVec,
                                double currentEnergyConsumption,
                                const TaskSet &taskDurOpt) const {
            for (int i = 0; i < N; i++) {
                if (tasks_.tasks_[i].deadline - responseTimeVec(i, 0) < 0 ||
                    (enableMaxComputationTimeRestrict &&
                     taskDurOpt[i].executionTimeOrg *
                             MaxComputationTimeRestrict <
                         taskDurOpt[i].executionTime) ||
                    taskDurOpt[i].executionTimeOrg >
                        taskDurOpt[i].executionTime)
                    return;
            }
            if (currentEnergyConsumption / weightEnergy < valueGlobalOpt) {
                // update globalOptVector
                vectorGlobalOpt =
                    GetParameterVD<double>(taskDurOpt, "executionTime");
                valueGlobalOpt = currentEnergyConsumption / weightEnergy;
            }
        }
        /**
         * @brief
         *
         * @param executionTimeVector (numberOfTasksNeedOptimize, 1)
         * @param H
         * @return Vector
         */
        gtsam::Vector evaluateError(
            const VectorDynamic &executionTimeVector,
            boost::optional<gtsam::Matrix &> H = boost::none) const override {
            BeginTimer("main_factor");
            TaskSetType taskDurOpt = tasks_;

            boost::function<gtsam::Matrix(const VectorDynamic &)> f2 =
                [this](const VectorDynamic &executionTimeVector) {
                    TaskSetType taskT = tasks_;
                    UpdateTaskSetExecutionTime(taskT.tasks_,
                                               executionTimeVector,
                                               lastTaskDoNotNeedOptimize);
                    VectorDynamic energys = EstimateEnergyTaskSet(taskT.tasks_);
                    if (!whether_ls) {
                        for (uint i = 0; i < taskT.tasks_.size(); i++) {
                            energys(i) = pow(energys(i), 0.5);
                        }
                    }

                    return energys;
                };

            boost::function<gtsam::Matrix(const VectorDynamic &)> f =
                [&taskDurOpt, &f2,
                 this](const VectorDynamic &executionTimeVector) {
                    UpdateTaskSetExecutionTime(taskDurOpt.tasks_,
                                               executionTimeVector,
                                               lastTaskDoNotNeedOptimize);
                    Schedul_Analysis r(taskDurOpt);
                    VectorDynamic responseTimeVec =
                        r.ResponseTimeOfTaskSet(responseTimeInitial);
                    VectorDynamic err = f2(executionTimeVector);

                    double currentEnergyConsumption =
                        EstimateEnergyTaskSet(taskDurOpt.tasks_).sum();
                    for (int i = 0; i < N; i++) {
                        // barrier function part
                        err(i, 0) += Barrier(taskDurOpt.tasks_[i].deadline -
                                             responseTimeVec(i, 0));
                        err(i, 0) +=
                            Barrier(taskDurOpt.tasks_[i].executionTime -
                                    taskDurOpt.tasks_[i].executionTimeOrg);
                        if (enableMaxComputationTimeRestrict)
                            err(i, 0) +=
                                Barrier(taskDurOpt.tasks_[i].executionTimeOrg *
                                            MaxComputationTimeRestrict -
                                        taskDurOpt.tasks_[i].executionTime);
                    }
                    UpdateGlobalVector(responseTimeVec,
                                       currentEnergyConsumption,
                                       taskDurOpt.tasks_);
                    return err;
                };

            VectorDynamic err;
            err = f(executionTimeVector);

            if (H) {
                if (exactJacobian == 1)
                    *H = NumericalDerivativeDynamicUpper(f, executionTimeVector,
                                                         deltaOptimizer, N);
                else if (exactJacobian == 0)
                    *H = NumericalDerivativeDynamic(f2, executionTimeVector,
                                                    deltaOptimizer, N);
                else if ((exactJacobian == -1)) {
                    *H = NumericalDerivativeDynamic(f2, executionTimeVector,
                                                    deltaOptimizer, N);
                    std::cout << "The old Jacobian is " << std::endl
                              << *H << std::endl;
                    int m = N;
                    int n = executionTimeVector.rows();
                    int maxIndex = 0;  // the index of variable
                    double maxElement = abs((*H)(m - n + maxIndex, maxIndex));
                    for (int i = 1; i < n; i++) {
                        if ((*H)(m - n + i, i) > maxElement) {
                            maxElement = abs((*H)(m - n + i, i));
                            maxIndex = i;
                        }
                    }
                    std::cout << "MaxIndex: " << maxIndex << std::endl;
                    for (int i = 0; i < n; i++) {
                        if (i != maxIndex) {
                            (*H)(m - n + i, i) = 0;
                        }
                    }
                }

                *H = *H * jacobianScale;
                if (debugMode == 1) {
                    std::cout << Color::blue << std::endl;
                    std::cout << "The current evaluation point is " << std::endl
                              << executionTimeVector << Color::def << std::endl;
                    std::cout << "The Jacobian is " << std::endl
                              << *H << std::endl;
                    // std::cout << "The approximated Jacobian is " << std::endl
                    //      << jacobian << std::endl;
                    std::cout << "The current error is " << std::endl
                              << err << std::endl
                              << std::endl
                              << err.norm() << std::endl
                              << std::endl;
                }
            }
            EndTimer("main_factor");
            return err;
        }
    };

    static void ClampComputationTime(TaskSetType &tasksSetType,
                                     int lastTaskDoNotNeedOptimize,
                                     VectorDynamic &responseTimeInitial,
                                     std::string roundType) {
        if (roundType == "none") return;
        TaskSet &tasks = tasksSetType.tasks_;
        for (uint i = 0; i < tasks.size(); i++)
            tasks[i].executionTime = int(tasks[i].executionTime);
        if (roundType == "rough") {
            return;
        } else if (roundType == "fine") {
            int N = tasks.size();

            std::vector<std::pair<int, double>> objectiveVec;
            objectiveVec.reserve(N);
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++) {
                objectiveVec.push_back(
                    std::make_pair(i, JacobianInEnergyItem(tasks, i)));
            }
            sort(objectiveVec.begin(), objectiveVec.end(), comparePair);

            int iterationNumber = 0;

            if (debugMode == 1) {
                std::cout << "before binary search, here is the task set"
                          << std::endl;
                for (int i = 0; i < N; i++) tasks[i].print();
            }

            // int left = 0, right = 0;
            while (objectiveVec.size() > 0) {
                int currentIndex = objectiveVec[0].first;

                // try to round up, if success, keep the loop; otherwise,
                // eliminate it and high priority tasks can be speeded up, if
                // necessary, by binary search
                int left = tasks[currentIndex].executionTime;
                int right = tasks[currentIndex].deadline;
                if (enableMaxComputationTimeRestrict) {
                    right = min(right, tasks[currentIndex].executionTimeOrg *
                                           MaxComputationTimeRestrict);
                }

                if (left > right) {
                    CoutError("left > right error in clamp!");
                }
                int rightOrg = right;
                bool schedulale_flag;
                while (left < right) {
                    int mid = ceil((left + right) / 2.0);

                    tasks[currentIndex].executionTime = mid;
                    Schedul_Analysis r(tasksSetType);
                    schedulale_flag = r.CheckSchedulability(responseTimeInitial,
                                                            debugMode == 1);
                    if ((not schedulale_flag) || not WithInBound(tasks)) {
                        right = mid - 1;
                    } else {
                        // comp(currentIndex, 0) = mid;
                        tasks[currentIndex].executionTime = mid;
                        left = mid;
                    }
                }

                // post processing, left=right is the value we want
                tasks[currentIndex].executionTime = left;
                objectiveVec.erase(objectiveVec.begin() + 0);
                if (left != rightOrg) {
                    // remove hp because they cannot be optimized anymore
                    for (int i = objectiveVec.size() - 1;
                         i > lastTaskDoNotNeedOptimize; i--) {
                        if (objectiveVec[i].first < currentIndex) {
                            objectiveVec.erase(objectiveVec.begin() + i);
                        }
                    }
                }

                iterationNumber++;
                if (iterationNumber > N) {
                    CoutWarning("iterationNumber error in Clamp!");
                    break;
                }
            };
        } else {
            std::cout << "input error in ClampComputationTime: " << roundType
                      << std::endl;
            throw;
        }
        return;
    }

    // get adjusting direction; 1 means increasing, -1 means decreasing
    static int GetDirection(Task &task) {
        double energy0 = EstimateEnergyTask(task);
        task.executionTime += deltaOptimizer;
        double energy1 = EstimateEnergyTask(task);
        if (energy1 <= energy0) {
            return 1;
        } else {
            return -1;
        }
    }
    /**
     * find the tasks that do not need to optimize;
     * i means i-th task do not need optimization,  while i+1, ..., N need
     * -1 means all tasks need optimization
     * N-1 means all tasks do not need optimization;
     *
     * Although Nasri's analysis cannot be verified to satisfy ''influence''
     *theorem or not, we assume it holds to enable faster optimization; the
     *results are always safe anyway.
     **/
    static int FindTaskDoNotNeedOptimize(
        const TaskSetType &tasks, int lastTaskDoNotNeedOptimize,
        VectorDynamic &computationTimeWarmStart, double eliminateTolIte) {
        BeginTimer(__func__);
        // update the tasks with the new optimal computationTimeVector
        TaskSetType tasksCurr = tasks;
        int N = tasks.tasks_.size();
        for (int i = N - 1; i > lastTaskDoNotNeedOptimize; i--) {
            if (((double)rand() / (RAND_MAX)) < SkipRateFindElimination) {
                continue;
            }
            // get adjusting direction; 1 means increasing, -1 means decreasing
            int direction = GetDirection(tasksCurr.tasks_[i]);
            tasksCurr.tasks_[i].executionTime += eliminateTolIte * direction;
            if (enableMaxComputationTimeRestrict &&
                tasksCurr.tasks_[i].executionTime - eliminateTolIte >
                    tasks.tasks_[i].executionTimeOrg *
                        MaxComputationTimeRestrict) {
                EndTimer(__func__);
                return i;
            }
            // we cannot use a more strict criteria in detecting schedulability,
            //  because it may trigger early detection of termination

            double tolerance = 0.0;
            Schedul_Analysis r(tasksCurr);
            bool schedulable = r.CheckSchedulability(computationTimeWarmStart,
                                                     false, tolerance);

            if (!schedulable) {
                EndTimer(__func__);
                return i;
            }

            tasksCurr.tasks_[i].executionTime -= eliminateTolIte;
        }
        EndTimer(__func__);
        return lastTaskDoNotNeedOptimize;
    }

    static VectorDynamic UnitOptimization(TaskSetType &tasks,
                                          int lastTaskDoNotNeedOptimize,
                                          VectorDynamic &initialEstimate,
                                          VectorDynamic &responseTimeInitial) {
        BeginTimer(__func__);
        int N = tasks.tasks_.size();

        // build the factor graph
        auto model = gtsam::noiseModel::Isotropic::Sigma(N, noiseModelSigma);
        gtsam::NonlinearFactorGraph graph;
        gtsam::Symbol key('a', 0);
        graph.emplace_shared<ComputationFactor>(
            key, tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, model);

        gtsam::Values initialEstimateFG;
        initialEstimateFG.insert(key, initialEstimate);

        gtsam::Values result;
        if (optimizerType == 1) {
            gtsam::DoglegParams params;
            // if (debugMode == 1)
            //     params.setVerbosityDL("VERBOSE");
            params.setDeltaInitial(deltaInitialDogleg);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setLinearSolverType(linearOptimizerType);
            gtsam::DoglegOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        } else if (optimizerType == 2) {
            gtsam::LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            // if (debugMode > 1 && debugMode < 5)
            params.setVerbosityLM(verbosityLM);
            params.setDiagonalDamping(setDiagonalDamping);
            // params.setVerbosityLM("TRYDELTA");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setMaxIterations(maxIterationsOptimizer);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setLinearSolverType(linearOptimizerType);
            gtsam::LevenbergMarquardtOptimizer optimizer(
                graph, initialEstimateFG, params);
            result = optimizer.optimize();
        } else if (optimizerType == 3) {
            gtsam::GaussNewtonParams params;
            if (debugMode == 1) params.setVerbosity("DELTA");
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setLinearSolverType(linearOptimizerType);
            gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimateFG,
                                                  params);
            result = optimizer.optimize();
        } else if (optimizerType == 4) {
            gtsam::NonlinearOptimizerParams params;
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setLinearSolverType(linearOptimizerType);
            if (debugMode == 1) params.setVerbosity("DELTA");
            gtsam::NonlinearConjugateGradientOptimizer optimizer(
                graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
        // if (debugMode == 1) {
        //     std::cout << "After optimization, the computation time vector is
        //     "
        //               << optComp << std::endl;
        // }

        EndTimer(__func__);
        return optComp;
    }

    /**
     * Perform optimization for one task set;
     * this function only performs optimization and elimination, it does not
     *change weights
     **/
    static double OptimizeTaskSetOneIte(TaskSetType &taskSetType) {
        int N = taskSetType.tasks_.size();

        // this function also checks schedulability
        Schedul_Analysis r(taskSetType);
        VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
        if (!r.CheckSchedulabilityDirect(responseTimeInitial)) return -2;

        VectorDynamic initialExecutionTime =
            GetParameterVD<int>(taskSetType, "executionTimeOrg");

        int lastTaskDoNotNeedOptimize = -1;

        lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(
            taskSetType, -1, responseTimeInitial, eliminateTol);

        // computationTimeVectorLocalOpt is always stored in tasks
        vectorGlobalOpt = initialExecutionTime;
        int numberOfIteration = 0;
        // eliminateTolIte must be inherited from one iteration to its next,
        // otherwise, circular elimination will occur
        double eliminateTolIte = eliminateTol;

        while (numberOfTasksNeedOptimize > 0) {
            VectorDynamic initialEstimateDuringOpt;
            initialEstimateDuringOpt.resize(numberOfTasksNeedOptimize, 1);
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                initialEstimateDuringOpt(i - lastTaskDoNotNeedOptimize - 1, 0) =
                    taskSetType.tasks_[i].executionTime;

            Schedul_Analysis r2(taskSetType);
            responseTimeInitial = r2.ResponseTimeOfTaskSet();
            // perform optimization

            auto variNew =
                UnitOptimization(taskSetType, lastTaskDoNotNeedOptimize,
                                 initialEstimateDuringOpt, responseTimeInitial);

            // formulate new computationTime
            UpdateTaskSetExecutionTime(taskSetType.tasks_, vectorGlobalOpt);
            // clamp with rough option seems to work better
            ClampComputationTime(taskSetType, lastTaskDoNotNeedOptimize,
                                 responseTimeInitial, clampTypeMiddle);
            // update vectorGlobalOpt to be the clamped version
            vectorGlobalOpt =
                GetParameterVD<double>(taskSetType.tasks_, "executionTime");
            valueGlobalOpt =
                EstimateEnergyTaskSet(taskSetType.tasks_).sum() / weightEnergy;
            if (debugMode == 1) {
                std::cout << "After clamp: " << std::endl
                          << GetParameterVD<double>(taskSetType.tasks_,
                                                    "executionTime")
                          << std::endl;
                Schedul_Analysis r(taskSetType);
                std::cout << "Execution time of tasks: " << std::endl;
                std::cout << vectorGlobalOpt << std::endl;
                VectorDynamic rtaTemp = r.ResponseTimeOfTaskSet();
                std::cout << "RTA after optimization: " << rtaTemp << std::endl;
            }

            // find variables to eliminate
            int adjustEliminateTolNum = 0;
            int lastTaskDoNotNeedOptimizeAfterOpt = -1;
            if (adjustEliminateMaxIte < 1) {
                CoutError("adjustEliminateMaxIte cannot be smaller than 1!");
            }

            while (adjustEliminateTolNum < adjustEliminateMaxIte) {
                lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(
                    taskSetType, lastTaskDoNotNeedOptimize, responseTimeInitial,
                    eliminateTolIte);
                if (lastTaskDoNotNeedOptimizeAfterOpt ==
                    lastTaskDoNotNeedOptimize)
                    eliminateTolIte *= eliminateStep;
                else
                    break;
                adjustEliminateTolNum++;
            }
            if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                break;

            lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;

            numberOfIteration++;
            if (numberOfIteration > min(N, elimIte)) {
                // CoutWarning("numberOfIteration reaches the maximum limits,
                // the algorithm decides to give up!");
                break;
            }
        }

        ClampComputationTime(taskSetType, -1, responseTimeInitial,
                             roundTypeInClamp);
        // performance evaluation
        Schedul_Analysis r2(taskSetType);
        if (r2.CheckSchedulability()) {
            if (debugMode == 1) {
                std::cout << "The task set is schedulable after optimization\n";
                // std::cout << std::endl;
                // std::cout << "The original task set is: " << std::endl;
                // for (int i = 0; i < N; i++) {
                //     std::cout << i << " ";
                //     taskSetType.tasks_[i].print();
                // }
            }
            TaskSet tasksInit = taskSetType.tasks_;
            UpdateTaskSetExecutionTime(tasksInit, initialExecutionTime);
            double initialEnergyCost = EstimateEnergyTaskSet(tasksInit).sum();
            double afterEnergyCost =
                EstimateEnergyTaskSet(taskSetType.tasks_).sum();
            // if (debugMode == 1)
            //     std::cout << "Actual objective function is" <<
            //     Energy_OptDAG<DAG_Nasri19,
            //     RTA_Nasri19>::RealObj(taskSetType.tasks_) << std::endl;
            if (debugMode == 1) {
                std::cout
                    << "Normalized objective function after optimization is "
                    << afterEnergyCost << std::endl;
            }
            if (debugMode >= 1) {
                double granularity =
                    GetParameterVD<double>(taskSetType, "executionTime")
                        .maxCoeff() *
                    3e-5;
                // verify whether elimination is successful
                Schedul_Analysis r1(taskSetType);
                if (r1.CheckSchedulability()) {
                    Task &taskLast =
                        taskSetType.tasks_[taskSetType.tasks_.size() - 1];
                    taskLast.executionTime += granularity;
                    Schedul_Analysis r2(taskSetType);
                    if (r2.CheckSchedulability()) {
                        if (enableMaxComputationTimeRestrict &&
                            taskLast.executionTime <
                                taskLast.executionTimeOrg *
                                    MaxComputationTimeRestrict) {
                            if (taskLast.executionTimeOrg / taskLast.period >
                                    0.03 &&
                                eliminateTolIte / taskLast.executionTime >
                                    0.015) {
                                //     CoutWarning(
                                //         "Elimination failed in final
                                //         verfication, \
                            // eliminateTolIte used before is " +
                                //         std::to_string(eliminateTolIte));
                            }
                        }
                    }
                    taskLast.executionTime -= granularity;
                }
            }
            if (runMode == "compare")
                return afterEnergyCost / weightEnergy;
            else if (runMode == "normal")
                return afterEnergyCost / initialEnergyCost;
            else {
                CoutError("Unrecognized runMode!!");
            }
        } else {
            std::cout << "Unfeasible after optimization!" << std::endl;
            return -1;
        }
        return -1;
    }

    /**
     * initialize all the global variables
     */
    static double OptimizeTaskSet(TaskSetType &taskSetType) {
        InitializeGlobalVector(taskSetType.tasks_.size());
        double eliminateTolRef = eliminateTol;

        double res = OptimizeTaskSetOneIte(taskSetType);
        std::cout << "After optimization: " << res << std::endl;
        // Some variables become 0, which actually means a failure
        if (isinf(res)) res = 10;
        eliminateTol = eliminateTolRef;
        return res;
    }
};
}  // namespace rt_num_opt