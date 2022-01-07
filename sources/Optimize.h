#pragma once

#include <chrono>
#include <math.h>

#pragma once

#include "Parameters.h"
#include <Eigen/Dense>
#include "Declaration.h"
#include "Tasks.h"
#include "RTA_LL.h"
#include "Energy.h"
#include "utils.h"
#include "FrequencyModel.h"

using namespace std;
using namespace gtsam;

#define numberOfTasksNeedOptimize (N - lastTaskDoNotNeedOptimize - 1)

double valueGlobalOpt = INT64_MAX;
VectorDynamic vectorGlobalOpt;

void InitializeGlobalVector(int N)
{
    vectorGlobalOpt.resize(N, 1);
    vectorGlobalOpt.setZero();
    valueGlobalOpt = INT64_MAX;
    TASK_NUMBER = N;
}
struct OptimizeResult
{
    double initialError;
    double optimizeError;
    VectorDynamic initialVariable;
    VectorDynamic optimizeVariable;
    OptimizeResult() : initialError(-1), optimizeError(-1)
    {
        ;
    }
    OptimizeResult(double ie, double oe, VectorDynamic iv, VectorDynamic ov) : initialError(ie),
                                                                               optimizeError(oe), initialVariable(iv), optimizeVariable(ov) {}
};

bool CheckExecutionTimeBound(VectorDynamic &exec, TaskSet &tasksOrg)
{
    if (not enableMaxComputationTimeRestrict)
        return true;
    for (uint i = 0; i < tasksOrg.size(); i++)
    {
        if (exec(i, 0) > tasksOrg[i].executionTime * 2)
            return false;
    }
    return true;
}

template <class Schedul_Analysis>
class Energy_Opt
{
public:
    // TODO: whether warmStart assume sustainable?
    class ComputationFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks_;
        int lastTaskDoNotNeedOptimize;
        VectorDynamic responseTimeInitial;
        int N;

        ComputationFactor(Key key, TaskSet &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic responseTimeInitial,
                          SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                    tasks_(tasks), lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
                                                    responseTimeInitial(responseTimeInitial)
        {
            N = tasks_.size();
        }

        void UpdateGlobalVector(VectorDynamic &responseTimeVec, double currentEnergyConsumption,
                                const TaskSet &taskDurOpt) const
        {
            for (int i = 0; i < N; i++)
            {
                if (tasks_[i].deadline - responseTimeVec(i, 0) < 0 ||
                    (enableMaxComputationTimeRestrict &&
                     taskDurOpt[i].executionTimeOrg * MaxComputationTimeRestrict <
                         taskDurOpt[i].executionTime))
                    return;
            }
            if (currentEnergyConsumption / weightEnergy < valueGlobalOpt)
            {
                // update globalOptVector
                vectorGlobalOpt = GetParameterVD<double>(taskDurOpt, "executionTime");
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
        Vector evaluateError(const VectorDynamic &executionTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {
            TaskSet taskDurOpt = tasks_;
            UpdateTaskSetExecutionTime(taskDurOpt, executionTimeVector, lastTaskDoNotNeedOptimize);
            VectorDynamic energyVec = EstimateEnergyTaskSet(taskDurOpt);

            VectorDynamic responseTimeVec = ResponseTimeOfTaskSet<Schedul_Analysis>(taskDurOpt, responseTimeInitial);

            boost::function<Matrix(const VectorDynamic &)> f2 =
                [this](const VectorDynamic &executionTimeVector)
            {
                TaskSet taskT = tasks_;
                UpdateTaskSetExecutionTime(taskT, executionTimeVector, lastTaskDoNotNeedOptimize);
                return EstimateEnergyTaskSet(taskT);
            };

            boost::function<Matrix(const VectorDynamic &)> f =
                [&responseTimeVec, &taskDurOpt, &f2, this](const VectorDynamic &executionTimeVector)
            {
                VectorDynamic err = f2(executionTimeVector);

                double currentEnergyConsumption = err.sum();
                for (int i = 0; i < N; i++)
                {
                    // barrier function part
                    err(i, 0) += Barrier(taskDurOpt[i].deadline - responseTimeVec(i, 0));
                    if (enableMaxComputationTimeRestrict)
                        err(i, 0) += Barrier(taskDurOpt[i].executionTimeOrg * MaxComputationTimeRestrict -
                                             taskDurOpt[i].executionTime);
                }
                UpdateGlobalVector(responseTimeVec, currentEnergyConsumption, taskDurOpt);
                return err;
            };

            VectorDynamic err;
            err = f(executionTimeVector);
            if (H)
            {
                if (exactJacobian)
                    *H = NumericalDerivativeDynamicUpper(f, executionTimeVector, deltaOptimizer, N);
                else
                    *H = NumericalDerivativeDynamicUpper(f2, executionTimeVector, deltaOptimizer, N);
                // *H = NumericalDerivativeDynamic(f2, executionTimeVector, deltaOptimizer, numberOfTasksNeedOptimize);
                // *H = jacobian;
                if (debugMode == 1)
                {
                    cout << endl;
                    cout << "The current evaluation point is " << endl
                         << executionTimeVector << endl;
                    cout << "The Jacobian is " << endl
                         << *H << endl;
                    // cout << "The approximated Jacobian is " << endl
                    //      << jacobian << endl;
                    cout << "The current error is " << endl
                         << err << endl
                         << endl
                         << err.norm() << endl
                         << endl;
                }

                if (debugMode == 1)
                {
                    cout << Color::green << "The response time and deadline for each task is: " << Color::def << endl;
                    for (int i = 0; i < N; i++)
                    {
                        cout << responseTimeVec(i) << ", " << taskDurOpt[i].deadline << endl;
                    }
                    err = f(executionTimeVector);
                }
            }

            return err;
        }
    };

    // ------------------
    // TODO: clamp turn 123.99 to 124 rather than 123
    static VectorDynamic ClampComputationTime(VectorDynamic comp, TaskSet &tasks, int lastTaskDoNotNeedOptimize,
                                              VectorDynamic &responseTimeInitial, string roundType)
    {
        if (roundType == "none")
            return comp;
        int n = comp.rows();
        for (int i = 0; i < n; i++)
            comp(i, 0) = int(comp(i, 0));
        if (roundType == "rough")
        {
            return comp;
        }
        else if (roundType == "fine")
        {
            int N = tasks.size();

            vector<pair<int, double>> objectiveVec;
            // objectiveVec.reserve(N);
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            {
                objectiveVec.push_back(make_pair(i, JacobianInClamp(tasks, comp, i)));
            }
            sort(objectiveVec.begin(), objectiveVec.end(), comparePair);
            int minEliminate = INT32_MAX;

            int iterationNumber = 0;
            TaskSet taskDuringOpt = tasks;
            UpdateTaskSetExecutionTime(taskDuringOpt, comp, lastTaskDoNotNeedOptimize);

            if (debugMode == 1)
            {
                cout << "before binary search, here is the task set" << endl;
                for (int i = 0; i < N; i++)
                    taskDuringOpt[i].print();
            }

            // int left = 0, right = 0;
            while (objectiveVec.size() > 0)
            {
                int currentIndex = objectiveVec[0].first;

                // try to round up, if success, keep the loop; otherwise, eliminate it and high priority tasks
                // can be speeded up, if necessary, by binary search
                int left = comp(currentIndex, 0);
                // int right = min(taskDuringOpt[currentIndex].deadline,
                //                 int(tasks[currentIndex].executionTime * computationBound));
                int right = taskDuringOpt[currentIndex].deadline;
                if (enableMaxComputationTimeRestrict)
                {
                    right = min(right, tasks[currentIndex].executionTime * computationBound);
                }
                // for (int j = 0; j < currentIndex; j++)
                //     right -= taskDuringOpt[j].executionTime;
                if (left > right)
                {
                    cout << "left > right error in clamp!" << endl;
                    throw;
                }
                int ref = comp(currentIndex, 0);

                while (left <= right)
                {
                    int mid = (left + right) / 2;

                    taskDuringOpt[currentIndex].executionTime = mid;

                    if ((not CheckSchedulability<Schedul_Analysis>(taskDuringOpt, responseTimeInitial)) ||
                        not WithInBound(tasks, taskDuringOpt))
                    {
                        taskDuringOpt[currentIndex].executionTime = ref;
                        comp(currentIndex, 0) = ref;
                        taskDuringOpt[currentIndex].executionTime = comp(currentIndex, 0);
                        minEliminate = currentIndex;
                        for (int i = objectiveVec.size() - 1; i > lastTaskDoNotNeedOptimize; i--)
                        {
                            if (objectiveVec[i].first <= minEliminate)
                            {
                                objectiveVec.erase(objectiveVec.begin() + i);
                            }
                        }
                        right = mid - 1;
                    }
                    else if (left == right)
                    {
                        minEliminate = currentIndex;
                        for (int i = objectiveVec.size() - 1; i > lastTaskDoNotNeedOptimize; i--)
                        {
                            if (objectiveVec[i].first <= minEliminate)
                            {
                                objectiveVec.erase(objectiveVec.begin() + i);
                            }
                        }
                        break;
                    }
                    else
                    {
                        comp(currentIndex, 0) = mid;
                        taskDuringOpt[currentIndex].executionTime = mid;
                        ref = mid;
                        left = mid + 1;
                    }
                }

                iterationNumber++;
                if (iterationNumber > N)
                {
                    cout << red << "iterationNumber error in Clamp!" << def << endl;
                    // throw;
                    break;
                }
            };
        }
        else
        {
            cout << "input error in ClampComputationTime" << endl;
            throw;
        }

        return comp;
    }

    /**
     * find the tasks that do not need to optimize;
     * i means i-th task do not need optimization,  while i+1, ..., N need
     * -1 means all tasks need optimization
     * N-1 means all tasks do not need optimization
     **/
    static int FindTaskDoNotNeedOptimize(const TaskSet &tasks, VectorDynamic computationTimeVector, int lastTaskDoNotNeedOptimize,
                                         VectorDynamic &computationTimeWarmStart, double eliminateTolIte)
    {
        // update the tasks with the new optimal computationTimeVector
        TaskSet tasksCurr = tasks;
        UpdateTaskSetExecutionTime(tasksCurr, computationTimeVector);
        int N = tasks.size();
        for (int i = N - 1; i >= 0; i--)
        {
            tasksCurr[i].executionTime += eliminateGranularity;

            // we cannot use a more strict criteria in detecting schedulability,
            //  because it may trigger early detection of termination

            // double rt = Schedul_Analysis::RTA_Common_Warm(computationTimeWarmStart(i, 0), tasksCurr, i);

            bool schedulable = CheckSchedulability<Schedul_Analysis>(tasksCurr,
                                                                     computationTimeWarmStart,
                                                                     debugMode == 1, 0);
            if ((!schedulable) ||
                (enableMaxComputationTimeRestrict &&
                 computationTimeVector(i, 0) + eliminateTolIte > tasks[i].executionTimeOrg * MaxComputationTimeRestrict))
                return i;
            // recover tasksCurr[i].executionTime
            tasksCurr[i].executionTime -= eliminateGranularity;
        }
        return -1;
    }

    static VectorDynamic UnitOptimization(TaskSet &tasks,
                                          int lastTaskDoNotNeedOptimize, VectorDynamic &initialEstimate,
                                          VectorDynamic &responseTimeInitial)
    {
        int N = tasks.size();

        // build the factor graph
        auto model = noiseModel::Isotropic::Sigma(N, noiseModelSigma);
        NonlinearFactorGraph graph;
        Symbol key('a', 0);
        graph.emplace_shared<ComputationFactor>(key, tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, model);

        Values initialEstimateFG;
        initialEstimateFG.insert(key, initialEstimate);

        // usually, when the change of variables between steps is smaller than 1,
        // we can already terminate; the corresponding minimal of relative error is
        // approximately 2e-3;

        Values result;
        if (optimizerType == 1)
        {
            DoglegParams params;
            // if (debugMode == 1)
            //     params.setVerbosityDL("VERBOSE");
            params.setDeltaInitial(deltaInitialDogleg);
            params.setRelativeErrorTol(relativeErrorTolerance);
            DoglegOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 2)
        {
            LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            if (debugMode > 1 && debugMode < 5)
                params.setVerbosityLM("SUMMARY");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 3)
        {
            GaussNewtonParams params;
            if (debugMode == 1)
                params.setVerbosity("DELTA");
            params.setRelativeErrorTol(relativeErrorTolerance);
            GaussNewtonOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 4)
        {
            NonlinearOptimizerParams params;
            params.setRelativeErrorTol(relativeErrorTolerance);
            if (debugMode == 1)
                params.setVerbosity("DELTA");
            NonlinearConjugateGradientOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
        if (debugMode == 1)
        {
            cout << "After optimization, the computation time vector is " << optComp << endl;
        }

        // if (debugMode == 1)
        //     cout << "After clamp, the computation time vector is " << optComp << endl;
        return optComp;
    }

    /**
     * Perform optimization for one task set;
     * this function only performs optimization and elimination, it does not change weights
     **/
    static double OptimizeTaskSetOneIte(TaskSet &tasks)
    {
        int N = tasks.size();

        // this function also checks schedulability
        VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<Schedul_Analysis>(tasks);
        if (!CheckSchedulabilityDirect(tasks, responseTimeInitial))
            return -2;

        VectorDynamic initialExecutionTime = GetParameterVD<int>(tasks, "executionTimeOrg");
        int lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(tasks, initialExecutionTime,
                                                                  -1, responseTimeInitial, eliminateTol);

        // its size is always N
        VectorDynamic computationTimeVectorLocalOpt = initialExecutionTime;
        vectorGlobalOpt = initialExecutionTime;
        int numberOfIteration = 0;
        // eliminateTolIte must be inherited from one iteration to its next, otherwise,
        // circular elimination will occur
        double eliminateTolIte = eliminateTol;

        while (numberOfTasksNeedOptimize > 0)
        {
            VectorDynamic initialEstimateDuringOpt;
            initialEstimateDuringOpt.resize(numberOfTasksNeedOptimize, 1);
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                initialEstimateDuringOpt(i - lastTaskDoNotNeedOptimize - 1, 0) =
                    computationTimeVectorLocalOpt(i, 0);

            responseTimeInitial = ResponseTimeOfTaskSet<Schedul_Analysis>(tasks);
            // perform optimization
            try
            {
                auto variNew = UnitOptimization(tasks, lastTaskDoNotNeedOptimize,
                                                initialEstimateDuringOpt, responseTimeInitial);
            }
            catch (...)
            {
                ;
            }

            // formulate new computationTime
            computationTimeVectorLocalOpt = vectorGlobalOpt;
            computationTimeVectorLocalOpt = ClampComputationTime(computationTimeVectorLocalOpt, tasks,
                                                                 lastTaskDoNotNeedOptimize,
                                                                 responseTimeInitial, roundTypeInClamp);
            UpdateTaskSetExecutionTime(tasks, computationTimeVectorLocalOpt);

            // find variables to eliminate
            int adjustEliminateTolNum = 0;
            int lastTaskDoNotNeedOptimizeAfterOpt;

            while (adjustEliminateTolNum < adjustEliminateMaxIte)
            {
                lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(
                    tasks,
                    computationTimeVectorLocalOpt, lastTaskDoNotNeedOptimize, responseTimeInitial, eliminateTolIte);
                if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                    eliminateTolIte *= eliminateStep;
                else
                    break;
                adjustEliminateTolNum++;
            }
            if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                break;

            if (debugMode == 1)
            {
                cout << "After one iteration, the computationTimeVectorLocalOpt is " << computationTimeVectorLocalOpt << endl;
                cout << "After one iteration, the vectorGlobalOpt is " << vectorGlobalOpt << endl;

                TaskSet tasks2 = tasks;
                for (int i = 0; i < N; i++)
                    tasks2[i].executionTime = computationTimeVectorLocalOpt(i, 0);
                VectorDynamic ttt = ResponseTimeOfTaskSet<Schedul_Analysis>(tasks2);
            }

            lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;

            numberOfIteration++;
            if (numberOfIteration > min(N, elimIte))
            {
                CoutWarning("numberOfIteration reaches the maximum limits, the algorithm decides to give up!");
                break;
            }
        }

        // performance evaluation
        if (CheckSchedulability<Schedul_Analysis>(tasks))
        {
            if (debugMode == 1)
            {
                cout << "The task set is schedulable after optimization\n";
                cout << endl;
                cout << "The original task set is: " << endl;
                for (int i = 0; i < N; i++)
                {
                    cout << i << " ";
                    tasks[i].print();
                }
            }
            if (debugMode == 1)
                cout << "computationTimeVectorLocalOpt before Clamp fine: " << computationTimeVectorLocalOpt << endl;
            computationTimeVectorLocalOpt = ClampComputationTime(computationTimeVectorLocalOpt, tasks, -1,
                                                                 responseTimeInitial, roundTypeInClamp);
            if (debugMode == 1)
                cout << "computationTimeVectorLocalOpt after Clamp fine: " << computationTimeVectorLocalOpt << endl;
            auto tasksInit = tasks;
            UpdateTaskSetExecutionTime(tasksInit, initialExecutionTime);
            double initialEnergyCost = EstimateEnergyTaskSet(tasksInit).sum();
            double afterEnergyCost = EstimateEnergyTaskSet(tasks).sum();
            if (debugMode == 1)
            {
                cout << "Normalized objective function after optimization is " << afterEnergyCost << endl;
                cout << "The variable after optimization is " << computationTimeVectorLocalOpt << endl;
            }
            if (debugMode >= 2)
            {
                // verify whether elimination is successful
                if (CheckSchedulability<Schedul_Analysis>(tasks))
                {
                    tasks[tasks.size() - 1].executionTime += 0.1;

                    if (CheckSchedulability<Schedul_Analysis>(tasks))
                    {
                        if (enableMaxComputationTimeRestrict &&
                            tasks[tasks.size() - 1].executionTime <
                                tasks[tasks.size() - 1].executionTimeOrg * MaxComputationTimeRestrict)
                            CoutWarning("Elimination failed in final verfication, \
                            eliminateTolIte used before is " +
                                        to_string(eliminateTolIte));
                    }
                    tasks[tasks.size() - 1].executionTime -= 0.1;
                }
            }
            if (runMode == "compare")
                return afterEnergyCost / weightEnergy;
            else if (runMode == "normal")
                return afterEnergyCost / initialEnergyCost;
            else
            {
                cout << red << "Unrecognized runMode!!" << def << endl;
                throw;
            }
        }
        else
        {
            cout << "Unfeasible after optimization!" << endl;
            return -1;
        }
        return -1;
    }

    /**
 * initialize all the global variables
 */
    static double OptimizeTaskSet(TaskSet &tasks)
    {
        InitializeGlobalVector(tasks.size());
        double eliminateTolRef = eliminateTol;

        double res = OptimizeTaskSetOneIte(tasks);
        // Some variables become 0, which actually means a failure
        if (isinf(res))
            res = 10;
        eliminateTol = eliminateTolRef;
        return res;
    }
};