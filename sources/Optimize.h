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

template <class TaskSetType, class Schedul_Analysis>
class Energy_Opt
{
public:
    // TODO: whether warmStart assume sustainable?
    class ComputationFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSetType tasks_;
        int lastTaskDoNotNeedOptimize;
        VectorDynamic responseTimeInitial;
        int N;

        ComputationFactor(Key key, TaskSetType &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic responseTimeInitial,
                          SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                    tasks_(tasks), lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
                                                    responseTimeInitial(responseTimeInitial)
        {
            N = tasks_.tasks_.size();
        }

        void UpdateGlobalVector(VectorDynamic &responseTimeVec, double currentEnergyConsumption,
                                const TaskSet &taskDurOpt) const
        {
            for (int i = 0; i < N; i++)
            {
                if (tasks_.tasks_[i].deadline - responseTimeVec(i, 0) < 0 ||
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
            TaskSetType taskDurOpt = tasks_;

            boost::function<Matrix(const VectorDynamic &)> f2 =
                [this](const VectorDynamic &executionTimeVector)
            {
                TaskSetType taskT = tasks_;
                UpdateTaskSetExecutionTime(taskT.tasks_, executionTimeVector, lastTaskDoNotNeedOptimize);
                return EstimateEnergyTaskSet(taskT.tasks_);
            };

            boost::function<Matrix(const VectorDynamic &)> f =
                [&taskDurOpt, &f2, this](const VectorDynamic &executionTimeVector)
            {
                UpdateTaskSetExecutionTime(taskDurOpt.tasks_, executionTimeVector, lastTaskDoNotNeedOptimize);
                Schedul_Analysis r(taskDurOpt);
                VectorDynamic responseTimeVec = r.ResponseTimeOfTaskSet(responseTimeInitial);
                VectorDynamic err = f2(executionTimeVector);

                double currentEnergyConsumption = err.sum();
                for (int i = 0; i < N; i++)
                {
                    // barrier function part
                    err(i, 0) += Barrier(taskDurOpt.tasks_[i].deadline - responseTimeVec(i, 0));
                    if (enableMaxComputationTimeRestrict)
                        err(i, 0) += Barrier(taskDurOpt.tasks_[i].executionTimeOrg * MaxComputationTimeRestrict -
                                             taskDurOpt.tasks_[i].executionTime);
                }
                UpdateGlobalVector(responseTimeVec, currentEnergyConsumption, taskDurOpt.tasks_);
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
            }

            return err;
        }
    };

    // ------------------
    // TODO: clamp turn 123.99 to 124 rather than 123
    static void ClampComputationTime(TaskSetType &tasksSetType, int lastTaskDoNotNeedOptimize,
                                     VectorDynamic &responseTimeInitial, string roundType)
    {
        if (roundType == "none")
            return;
        TaskSet &tasks = tasksSetType.tasks_;
        for (uint i = 0; i < tasks.size(); i++)
            tasks[i].executionTime = int(tasks[i].executionTime);
        if (roundType == "rough")
        {
            return;
        }
        else if (roundType == "fine")
        {
            int N = tasks.size();

            vector<pair<int, double>> objectiveVec;
            objectiveVec.reserve(N);
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            {
                objectiveVec.push_back(make_pair(i, JacobianInEnergyItem(tasks, i)));
            }
            sort(objectiveVec.begin(), objectiveVec.end(), comparePair);

            int iterationNumber = 0;

            if (debugMode == 1)
            {
                cout << "before binary search, here is the task set" << endl;
                for (int i = 0; i < N; i++)
                    tasks[i].print();
            }

            // int left = 0, right = 0;
            while (objectiveVec.size() > 0)
            {
                int currentIndex = objectiveVec[0].first;

                // try to round up, if success, keep the loop; otherwise, eliminate it and high priority tasks
                // can be speeded up, if necessary, by binary search
                int left = tasks[currentIndex].executionTime;
                int right = tasks[currentIndex].deadline;
                if (enableMaxComputationTimeRestrict)
                {
                    right = min(right, tasks[currentIndex].executionTimeOrg * MaxComputationTimeRestrict);
                }

                if (left > right)
                {
                    CoutError("left > right error in clamp!");
                }
                int rightOrg = right;
                bool schedulale_flag;
                while (left < right)
                {
                    int mid = ceil((left + right) / 2.0);

                    tasks[currentIndex].executionTime = mid;
                    Schedul_Analysis r(tasksSetType);
                    schedulale_flag = r.CheckSchedulability(
                        responseTimeInitial, debugMode == 1);
                    if ((not schedulale_flag) ||
                        not WithInBound(tasks))
                    {
                        right = mid - 1;
                    }
                    else
                    {
                        // comp(currentIndex, 0) = mid;
                        tasks[currentIndex].executionTime = mid;
                        left = mid;
                    }
                }

                // post processing, left=right is the value we want
                tasks[currentIndex].executionTime = left;
                objectiveVec.erase(objectiveVec.begin() + 0);
                if (left != rightOrg)
                {
                    // remove hp because they cannot be optimized anymore
                    for (int i = objectiveVec.size() - 1; i > lastTaskDoNotNeedOptimize; i--)
                    {
                        if (objectiveVec[i].first < currentIndex)
                        {
                            objectiveVec.erase(objectiveVec.begin() + i);
                        }
                    }
                }

                iterationNumber++;
                if (iterationNumber > N)
                {
                    CoutWarning("iterationNumber error in Clamp!");
                    break;
                }
            };
        }
        else
        {
            cout << "input error in ClampComputationTime: " << roundType << endl;
            throw;
        }
        return;
    }

    /**
     * find the tasks that do not need to optimize;
     * i means i-th task do not need optimization,  while i+1, ..., N need
     * -1 means all tasks need optimization
     * N-1 means all tasks do not need optimization
     **/
    static int FindTaskDoNotNeedOptimize(const TaskSetType &tasks, int lastTaskDoNotNeedOptimize,
                                         VectorDynamic &computationTimeWarmStart, double eliminateTolIte)
    {
        // update the tasks with the new optimal computationTimeVector
        TaskSetType tasksCurr = tasks;
        int N = tasks.tasks_.size();
        for (int i = N - 1; i >= 0; i--)
        {
            tasksCurr.tasks_[i].executionTime += eliminateTolIte;

            // we cannot use a more strict criteria in detecting schedulability,
            //  because it may trigger early detection of termination

            // double rt = Schedul_Analysis::RTA_Common_Warm(computationTimeWarmStart(i, 0), tasksCurr, i);
            double tolerance = 0.0;
            Schedul_Analysis r(tasksCurr);
            bool schedulable = r.CheckSchedulability(
                computationTimeWarmStart,
                debugMode == 1, tolerance);
            // bool schedulable = Schedul_Analysis::CheckSchedulability(tasksCurr,
            //                                                          computationTimeWarmStart,
            //                                                          debugMode == 1, tolerance);
            if ((!schedulable) ||
                (enableMaxComputationTimeRestrict &&
                 tasksCurr.tasks_[i].executionTime - eliminateTolIte > tasks.tasks_[i].executionTimeOrg * MaxComputationTimeRestrict))

                // double rt = Schedul_Analysis::RTA_Common_Warm(computationTimeWarmStart(i, 0), tasksCurr, i);
                // if (abs(rt - tasks[i].deadline) <= tolerance || rt > tasks[i].deadline ||
                //     tasksCurr[i].executionTime - eliminateTolIte + tolerance > tasks[i].executionTimeOrg * MaxComputationTimeRestrict)
                return i;
            // recover tasksCurr[i].executionTime
            tasksCurr.tasks_[i].executionTime -= eliminateTolIte;
        }
        return -1;
    }

    static VectorDynamic UnitOptimization(TaskSetType &tasks,
                                          int lastTaskDoNotNeedOptimize, VectorDynamic &initialEstimate,
                                          VectorDynamic &responseTimeInitial)
    {
        int N = tasks.tasks_.size();

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
    static double OptimizeTaskSetOneIte(TaskSetType &taskSetType)
    {
        int N = taskSetType.tasks_.size();

        // this function also checks schedulability
        Schedul_Analysis r(taskSetType);
        VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
        if (!r.CheckSchedulabilityDirect(responseTimeInitial))
            return -2;

        VectorDynamic initialExecutionTime = GetParameterVD<int>(taskSetType, "executionTimeOrg");
        int lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(taskSetType,
                                                                  -1, responseTimeInitial, eliminateTol);

        // computationTimeVectorLocalOpt is always stored in tasks
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
                    taskSetType.tasks_[i].executionTime;
            Schedul_Analysis r2(taskSetType);
            responseTimeInitial = r2.ResponseTimeOfTaskSet();
            // perform optimization
            if (exactJacobian)
            {
                try
                {
                    auto variNew = UnitOptimization(taskSetType, lastTaskDoNotNeedOptimize,
                                                    initialEstimateDuringOpt, responseTimeInitial);
                }
                catch (...)
                {
                    ;
                }
            }
            else
            {
                auto variNew = UnitOptimization(taskSetType, lastTaskDoNotNeedOptimize,
                                                initialEstimateDuringOpt, responseTimeInitial);
            }

            // formulate new computationTime
            UpdateTaskSetExecutionTime(taskSetType.tasks_, vectorGlobalOpt);
            // clamp with rough option seems to work better
            ClampComputationTime(taskSetType,
                                 lastTaskDoNotNeedOptimize,
                                 responseTimeInitial, "rough");
            if (debugMode == 1)
                cout << "After clamp: " << endl
                     << GetParameterVD<double>(taskSetType.tasks_, "executionTime") << endl;
            // find variables to eliminate
            int adjustEliminateTolNum = 0;
            int lastTaskDoNotNeedOptimizeAfterOpt;

            while (adjustEliminateTolNum < adjustEliminateMaxIte)
            {
                lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(
                    taskSetType, lastTaskDoNotNeedOptimize, responseTimeInitial, eliminateTolIte);
                if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                    eliminateTolIte *= eliminateStep;
                else
                    break;
                adjustEliminateTolNum++;
            }
            if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                break;

            lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;

            numberOfIteration++;
            if (numberOfIteration > min(N, elimIte))
            {
                CoutWarning("numberOfIteration reaches the maximum limits, the algorithm decides to give up!");
                break;
            }
        }
        ClampComputationTime(taskSetType,
                             -1,
                             responseTimeInitial, roundTypeInClamp);
        // performance evaluation
        Schedul_Analysis r2(taskSetType);
        if (r2.CheckSchedulability())
        {
            if (debugMode == 1)
            {
                cout << "The task set is schedulable after optimization\n";
                cout << endl;
                cout << "The original task set is: " << endl;
                for (int i = 0; i < N; i++)
                {
                    cout << i << " ";
                    taskSetType.tasks_[i].print();
                }
            }
            TaskSet tasksInit = taskSetType.tasks_;
            UpdateTaskSetExecutionTime(tasksInit, initialExecutionTime);
            double initialEnergyCost = EstimateEnergyTaskSet(tasksInit).sum();
            double afterEnergyCost = EstimateEnergyTaskSet(taskSetType.tasks_).sum();
            if (debugMode == 1)
            {
                cout << "Normalized objective function after optimization is " << afterEnergyCost << endl;
            }
            if (debugMode >= 1)
            {
                double granularity = GetParameterVD<double>(taskSetType, "executionTime").maxCoeff() * 3e-5;
                // verify whether elimination is successful
                Schedul_Analysis r1(taskSetType);
                if (r1.CheckSchedulability())
                // if (Schedul_Analysis::CheckSchedulability(taskSetType))
                {
                    Task &taskLast = taskSetType.tasks_[taskSetType.tasks_.size() - 1];
                    taskLast.executionTime += granularity;
                    Schedul_Analysis r2(taskSetType);
                    if (r2.CheckSchedulability())
                    {

                        if (enableMaxComputationTimeRestrict &&
                            taskLast.executionTime <
                                taskLast.executionTimeOrg * MaxComputationTimeRestrict)
                        {
                            if (taskLast.executionTimeOrg /
                                        taskLast.period >
                                    0.03 &&
                                eliminateTolIte / taskLast.executionTime > 0.015)
                            {
                                CoutWarning("Elimination failed in final verfication, \
                            eliminateTolIte used before is " +
                                            to_string(eliminateTolIte));
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
            else
            {
                CoutError("Unrecognized runMode!!");
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
    static double OptimizeTaskSet(TaskSetType &taskSetType)
    {
        InitializeGlobalVector(taskSetType.tasks_.size());
        double eliminateTolRef = eliminateTol;

        double res = OptimizeTaskSetOneIte(taskSetType);
        // Some variables become 0, which actually means a failure
        if (isinf(res))
            res = 10;
        eliminateTol = eliminateTolRef;
        return res;
    }
};