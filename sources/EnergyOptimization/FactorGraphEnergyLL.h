#pragma once
#include <chrono>
#include <math.h>

#include <Eigen/Dense>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "sources/Utils/Parameters.h"
#include "sources/MatirxConvenient.h"
#include "sources/TaskModel/Tasks.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/EnergyOptimization/Energy.h"
#include "sources/Utils/utils.h"
#include "sources/EnergyOptimization/FrequencyModel.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/GlobalVariables.h"
namespace rt_num_opt
{
    struct FactorGraphEnergyLL
    {
        static VectorDynamic ExtractResults(const gtsam::Values &result, const TaskSet &tasks)
        {
            VectorDynamic executionTimes = GenerateVectorDynamic(result.size());
            for (uint i = 0; i < result.size(); i++)
            {
                if (result.exists(GenerateControlKey(i, "executionTime")))
                {
                    executionTimes(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "executionTime"))(0, 0);
                }
                else
                {
                    CoutError("Key not found in ExtractResults!");
                }
            }
            return executionTimes;
        }

        static void TryUpdateGlobalVector(const TaskSet tasks, const gtsam::Values x)
        {
            if (x.size() != 20)
                return;
            // try updating global vector
            TaskSet tasksTry = tasks;
            VectorDynamic execTry = ExtractResults(x, tasks);
            UpdateTaskSetExecutionTime(tasksTry, execTry);
            double currVal = RealObj(tasksTry);
            bool schedulable = true;
            if (currVal < valueGlobalOpt)
            {
                RTA_LL r(tasksTry);
                if (r.CheckSchedulability(false))
                {
                    for (uint i = 0; i < tasksTry.size(); i++)
                    {
                        if (enableMaxComputationTimeRestrict && tasksTry[i].executionTime > MaxComputationTimeRestrict * tasksTry[i].executionTimeOrg + 1e-3)
                        {
                            schedulable = false;
                            break;
                        }
                    }
                }
                else
                {
                    schedulable = false;
                }
            }
            if (schedulable && currVal < valueGlobalOpt)
            {
                valueGlobalOpt = currVal;
                vectorGlobalOpt = execTry;
            }
        }

        class RTARelatedFactor : public gtsam::NoiseModelFactor
        {
        public:
            TaskSet tasks;
            int index;
            VectorDynamic rtaBase;
            int dimension;
            std::vector<gtsam::Symbol> keyVec;
            LambdaMultiKey f_with_RTA;

            RTARelatedFactor(std::vector<gtsam::Symbol> &keyVec, TaskSet &tasks, int index, VectorDynamic rtaBase,
                             gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor(model, keyVec), tasks(tasks), index(index), rtaBase(rtaBase), dimension(keyVec.size()), keyVec(keyVec)
            {
                f_with_RTA = [tasks, index, rtaBase](const gtsam::Values &x)
                {
                    BeginTimer("f_with_RTA");
                    VectorDynamic error = GenerateVectorDynamic(1);
                    TaskSet tasksCurr = tasks;
                    UpdateTaskSetExecutionTime(tasksCurr, FactorGraphEnergyLL::ExtractResults(x, tasks));
                    RTA_LL r(tasksCurr);
                    double rta = r.RTA_Common_Warm(rtaBase(index), index);
                    error(0) = HingeLoss(min(tasksCurr[index].period, tasksCurr[index].deadline) - rta);

                    eliminationRecordGlobal.AdjustEliminationError(error(0), index, EliminationType::RTA);
                    EndTimer("f_with_RTA");
                    return error;
                };
            }
            /* no need to optimize if it contains no keys */
            bool active(const gtsam::Values &c) const override
            {
                return keyVec.size() != 0;
            }
            gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                          boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
            {
                BeginTimer("RTARelatedFactor_unwhitenedError");
                if (H)
                {
                    for (int i = 0; i < dimension; i++)
                    {
                        if (exactJacobian)
                        {
                            NormalErrorFunction1D f =
                                [x, i, this](const VectorDynamic xi)
                            {
                                gtsam::Symbol a = keyVec.at(i);
                                gtsam::Values xx = x;
                                xx.update(a, xi);
                                return f_with_RTA(xx);
                            };
                            (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                        }

                        else
                            (*H)[i] = GenerateVectorDynamic(1);
                    }
                    if (debugMode == 1)
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        std::cout << Color::blue;
                        // PrintControlValues(x);
                        // x.print();
                        std::cout << Color::def;
                    }
                }
                // uint t = x.size();
                TryUpdateGlobalVector(tasks, x);
                EndTimer("RTARelatedFactor_unwhitenedError");
                return f_with_RTA(x);
            }
        };

        static RTARelatedFactor
        GenerateRTARelatedFactor(TaskSet &tasks, int index,
                                 VectorDynamic &rtaBase)
        {

            std::vector<gtsam::Symbol> keys;
            keys.reserve(index);
            for (int i = 0; i <= index; i++)
            {
                keys.push_back(GenerateControlKey(i, "executionTime"));
            }

            VectorDynamic sigma = GenerateVectorDynamic(1);
            sigma << noiseModelSigma / weightSchedulability;
            auto model = gtsam::noiseModel::Diagonal::Sigmas(sigma);
            // return MultiKeyFactor(keys, f, model);
            return RTARelatedFactor(keys, tasks, index, rtaBase, model);
        }

        static MultiKeyFactor GenerateEliminationLLFactor(TaskSet tasks,
                                                          int index, double rtaAtIndex)
        {
            std::vector<gtsam::Symbol> keys;
            for (int i = 0; i <= index; i++)
            {
                keys.push_back(GenerateControlKey(i, "executionTime"));
            }
            LambdaMultiKey f = [tasks, index, rtaAtIndex](const gtsam::Values &x)
            {
                VectorDynamic error = GenerateVectorDynamic(1);
                TaskSet tasksCurr = tasks;
                VectorDynamic executionTimeVecCurr = FactorGraphEnergyLL::ExtractResults(x, tasks).block(0, 0, index + 1, 1);

                MatrixDynamic coeff = executionTimeVecCurr.transpose();

                for (int i = 0; i < index; i++)
                {
                    coeff(i) = ceil(rtaAtIndex / tasks[i].period);
                }
                coeff(index) = 1;
                error(0) = rtaAtIndex - (coeff * executionTimeVecCurr)(0, 0);
                // std::cout << "Real error: " << RealObj(tasks) << std::endl;
                return error;
            };
            auto model = gtsam::noiseModel::Constrained::All(1);
            return MultiKeyFactor(keys, f, gtsam::noiseModel::Constrained::All(1));
        }

        static MultiKeyFactor GenerateLockLLFactor(TaskSet tasks,
                                                   int index)
        {
            std::vector<gtsam::Symbol> keys;
            keys.push_back(GenerateControlKey(index, "executionTime"));

            LambdaMultiKey f = [tasks, index](const gtsam::Values &x)
            {
                VectorDynamic error = GenerateVectorDynamic(1);
                TaskSet tasksCurr = tasks;
                VectorDynamic executionTimeVecCurr = x.at<VectorDynamic>(GenerateControlKey(index, "executionTime"));
                error(0) = executionTimeVecCurr(0, 0) - tasks[index].executionTime;

                return error;
            };
            auto model = gtsam::noiseModel::Constrained::All(1);
            return MultiKeyFactor(keys, f, gtsam::noiseModel::Constrained::All(1));
        }

        class EnergyFactor : public gtsam::NoiseModelFactor1<VectorDynamic>
        {
        public:
            int taskIndex;
            Task task_;

            /**
             * @brief Construct a new Inequality Factor 1 D object,
             *  mainly used in derived class because f is not defined
             */
            EnergyFactor(gtsam::Key key, Task &task, int index,
                         gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<VectorDynamic>(model, key), taskIndex(index),
                                                          task_(task)
            {
            }

            gtsam::Vector evaluateError(const VectorDynamic &x,
                                        boost::optional<gtsam::Matrix &> H = boost::none) const override
            {
                boost::function<gtsam::Matrix(const VectorDynamic &)> f =
                    [this](const VectorDynamic &executionTimeVector)
                {
                    Task taskCurr = task_;
                    taskCurr.executionTime = executionTimeVector(0, 0);
                    VectorDynamic err = GenerateVectorDynamic1D(pow(1.0 / taskCurr.period *
                                                                        EstimateEnergyTask(taskCurr),
                                                                    1));
                    // if (debugMode == 1)
                    // {
                    //     std::cout << "Task " << task_.id << "'s Energy cost is is " << err << std::endl;
                    // }
                    return err;
                };
                VectorDynamic err = f(x);
                if (H)
                {
                    *H = NumericalDerivativeDynamic(f, x, deltaOptimizer, 1);
                    if ((*H)(0, 0) == 0)
                    {
                        // int a = 1;
                    }
                    if (gradientModify != 0)
                    {
                        *H = *H * (1 + 0.01 * taskIndex * gradientModify);
                    }
                }
                return err;
            }
        };

        static gtsam::NonlinearFactorGraph BuildControlGraph(TaskSet tasks)
        {
            VectorDynamic rtaBase = RTALLVector(tasks);
            gtsam::NonlinearFactorGraph graph;
            auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
            auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
            // auto modelPunishmentHard = gtsam::noiseModel::Constrained::All(1);

            for (uint i = 0; i < tasks.size(); i++)
            {
                // energy factor
                graph.emplace_shared<EnergyFactor>(GenerateControlKey(i, "executionTime"), tasks[i], i, modelNormal);

                // add executionTime min/max limits
                graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "executionTime"), tasks[i].executionTimeOrg, i, modelPunishmentSoft1);
                double limit = min(tasks[i].deadline, tasks[i].period);
                if (enableMaxComputationTimeRestrict)
                {
                    limit = min(limit, tasks[i].executionTimeOrg * MaxComputationTimeRestrict);
                }
                graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "executionTime"), limit,
                                                          i, modelPunishmentSoft1);

                if (eliminationRecordGlobal[i].type == EliminationType::Not)
                {
                    // RTA factor
                    graph.add(GenerateRTARelatedFactor(tasks, i, rtaBase));
                }
                else if (eliminationRecordGlobal[i].type == EliminationType::RTA)
                {
                    graph.add(GenerateEliminationLLFactor(tasks, i, rtaBase(i)));
                }
                else if (eliminationRecordGlobal[i].type == EliminationType::Bound)
                {
                    graph.add(GenerateLockLLFactor(tasks, i));
                    // RTA factor
                    graph.add(GenerateRTARelatedFactor(tasks, i, rtaBase));
                }
            }
            graph.add(GenerateRTARelatedFactor(tasks, tasks.size() - 1, rtaBase));
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
        static gtsam::NonlinearFactorGraph BuildGraphForC(TaskSet &tasks)
        {
            gtsam::NonlinearFactorGraph graph;
            auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);

            for (uint i = 0; i < tasks.size(); i++)
            {
                // energy factor
                graph.emplace_shared<EnergyFactor>(GenerateControlKey(i, "executionTime"), tasks[i], i, modelNormal);
            }
            return graph;
        }
        /**
         * @brief To analyze Jacobian for constraints
         *
         * @param tasks
         * @return NonlinearFactorGraph
         */
        static gtsam::NonlinearFactorGraph BuildGraphForJ(TaskSet &tasks)
        {
            VectorDynamic rtaBase = RTALLVector(tasks);
            gtsam::NonlinearFactorGraph graph;
            auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
            // auto modelPunishmentHard = noiseModel::Constrained::All(1);

            for (uint i = 0; i < tasks.size(); i++)
            {
                // add executionTime min/max limits
                graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "executionTime"), tasks[i].executionTimeOrg, i, modelPunishmentSoft1);
                double limit = min(tasks[i].deadline, tasks[i].period);
                if (enableMaxComputationTimeRestrict)
                {
                    limit = min(limit, tasks[i].executionTimeOrg * MaxComputationTimeRestrict);
                }
                graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "executionTime"), limit,
                                                          i, modelPunishmentSoft1);

                if (eliminationRecordGlobal[i].type == EliminationType::Not)
                {
                    // RTA factor
                    graph.add(GenerateRTARelatedFactor(tasks, i, rtaBase));
                }
                else if (eliminationRecordGlobal[i].type == EliminationType::RTA)
                {
                    graph.add(GenerateEliminationLLFactor(tasks, i, rtaBase(i)));
                }
                else if (eliminationRecordGlobal[i].type == EliminationType::Bound)
                {
                    graph.add(GenerateLockLLFactor(tasks, i));
                    // RTA factor
                    graph.add(GenerateRTARelatedFactor(tasks, i, rtaBase));
                }
            }
            graph.add(GenerateRTARelatedFactor(tasks, tasks.size() - 1, rtaBase));
            return graph;
        }

        static gtsam::Values
        GenerateInitialFG(TaskSet tasks)
        {
            gtsam::Values initialEstimateFG;
            for (uint i = 0; i < tasks.size(); i++)
            {
                initialEstimateFG.insert(GenerateControlKey(i, "executionTime"),
                                         GenerateVectorDynamic1D(tasks[i].executionTime));
            }
            return initialEstimateFG;
        }

        static double FindEliminatedVariables(TaskSet &tasks, std::vector<bool> &maskForElimination,
                                              bool &whether_new_eliminate, double disturb = disturb_init)
        {
            BeginTimer(__func__);
            whether_new_eliminate = false;
            if (debugMode == 1)
            {
                std::cout << GetParameterVD<double>(tasks, "executionTime") << std::endl;
            }

            RTA_LL r(tasks);
            VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
            // bool whether_new_eliminate = false;
            while (!whether_new_eliminate && disturb <= disturb_max)
            {
                for (uint i = 0; i < tasks.size(); i++)
                {
                    if (maskForElimination[i])
                        continue;
                    tasks[i].executionTime += disturb;
                    RTA_LL r1(tasks);
                    // VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet(rtaBase);
                    if (!r1.CheckSchedulability(1 == debugMode) || (enableMaxComputationTimeRestrict &&
                                                                    tasks[i].executionTime > tasks[i].executionTimeOrg * MaxComputationTimeRestrict))
                    // TODO: more analytic way
                    {
                        if (!maskForElimination[i])
                            whether_new_eliminate = true;
                        maskForElimination[i] = true;
                    }
                    else
                    {
                        maskForElimination[i] = false;
                    }
                    tasks[i].executionTime -= disturb;
                }
                if (!whether_new_eliminate)
                    disturb *= disturb_step;
                if (debugMode == 1)
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    for (auto a : maskForElimination)
                        std::cout << a << ", ";
                    std::cout << std::endl;
                }
            }
            EndTimer(__func__);
            return disturb;
        }

        static double RealObj(const TaskSet tasks)
        {
            return EstimateEnergyTaskSet(tasks).sum() / weightEnergy;
        }
    };
} // namespace rt_num_opt