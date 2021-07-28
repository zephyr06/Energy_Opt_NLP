#pragma once

#include <chrono>
#include <math.h>

#pragma once

#include <Eigen/Dense>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "Tasks.h"
#include "ResponseTimeAnalysis.h"
#include "Energy.h"
#include "Parameters.h"
#include "WAP/RTA_WAP.h"

using namespace std;
using namespace gtsam;

#define numberOfTasksNeedOptimize (N - lastTaskDoNotNeedOptimize - 1)

double valueGlobalOpt = INT64_MAX;
VectorDynamic vectorGlobalOpt;

/**
 * barrier function for the optimization
 **/
double Barrier(double x)
{
    if (x > 0)
        // return pow(x, 2);
        return weightLogBarrier * log(x);
    else if (x < 0)
    {
        if (TASK_NUMBER == 0)
        {
            cout << red << "Please set TASK_NUMBER!" << def << endl;
            throw;
        }
        return punishmentInBarrier * pow(10, TASK_NUMBER - 3) * pow(1 - x, 1);
    }
    else // it basically means x=0
        return weightLogBarrier *
               log(x + toleranceBarrier);
}

MatrixDynamic NumericalDerivativeDynamic(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                         VectorDynamic x, double deltaOptimizer, int mOfJacobian)
{
    int n = x.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);

    for (int i = 0; i < n; i++)
    {
        VectorDynamic xDelta = x;
        xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
        VectorDynamic resPlus;
        resPlus.resize(mOfJacobian, 1);
        resPlus = h(xDelta);
        // cout << "resPlus" << endl
        //      << resPlus << endl;

        xDelta(i, 0) = xDelta(i, 0) - 2 * deltaOptimizer;
        VectorDynamic resMinus;
        resMinus.resize(mOfJacobian, 1);
        resMinus = h(xDelta);

        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
        }
    }
    return jacobian;
}

MatrixDynamic NumericalDerivativeDynamicUpper(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                              VectorDynamic x, double deltaOptimizer, int mOfJacobian)
{
    int n = x.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);
    VectorDynamic currErr = h(x);

    for (int i = 0; i < n; i++)
    {
        VectorDynamic xDelta = x;
        xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
        VectorDynamic resPlus;
        resPlus.resize(mOfJacobian, 1);
        resPlus = h(xDelta);
        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
        }
    }
    return jacobian;
}

class ComputationFactor : public NoiseModelFactor1<VectorDynamic>
{
public:
    TaskSet tasks_;
    int lastTaskDoNotNeedOptimize;
    // int numberOfTasksNeedOptimize;
    VectorDynamic responseTimeInitial;
    int N;

    ComputationFactor(Key key, TaskSet &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic responseTimeInitial,
                      SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                tasks_(tasks), lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
                                                responseTimeInitial(responseTimeInitial)
    {
        N = tasks_.size();
        TASK_NUMBER = N;
        // numberOfTasksNeedOptimize = N - lastTaskDoNotNeedOptimize - 1;
    }

    Vector evaluateError(const VectorDynamic &executionTimeVector, boost::optional<Matrix &> H = boost::none) const override
    {

        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            bool flagSchedulable = true;
            double currentEnergyConsumption = 0;

            VectorDynamic err;
            err.resize(N, 1);
            TaskSet taskSetCurr_ = tasks_;
            UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);

            vector<Task> hpTasks;
            for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
            {
                hpTasks.push_back(taskSetCurr_[i]);
            }
            // cout << "The response time and deadline for each task is: " << endl;
            for (int i = 0; i < N; i++)
            {
                // energy part
                double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                err(i, 0) = 1.0 / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                currentEnergyConsumption += err(i, 0);
                // barrier function part
                double responseTime = ResponseTimeWAP(taskSetCurr_, A_Global, P_Global, i, responseTimeInitial(i, 0));
                // cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                err(i, 0) += Barrier(tasks_[i].deadline - responseTime);
                if (tasks_[i].deadline - responseTime < 0)
                    flagSchedulable = false;
            }

            // check schedulability
            if (flagSchedulable && currentEnergyConsumption / weightEnergy < valueGlobalOpt)
            {
                // update globalOptVector
                valueGlobalOpt = currentEnergyConsumption / weightEnergy;
                for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                    vectorGlobalOpt(i, 0) = executionTimeVector(i - lastTaskDoNotNeedOptimize - 1, 0);
                // if (debugMode == 1)
                //     cout << "vectorGlobalOpt is " << vectorGlobalOpt << endl;
            }

            return err;
        };

        boost::function<Matrix(const VectorDynamic &)> f2 =
            [this](const VectorDynamic &executionTimeVector)
        {
            VectorDynamic err;
            err.resize(N, 1);
            TaskSet taskSetCurr_ = tasks_;
            UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);

            // cout << "The response time and deadline for each task is: " << endl;
            for (int i = 0; i < N; i++)
            {
                // energy part
                double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                err(i, 0) = 1.0 / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
            }
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
                TaskSet taskSetCurr_ = tasks_;
                UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);
                vector<Task> hpTasks;
                for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
                {
                    hpTasks.push_back(taskSetCurr_[i]);
                }
                cout << "The response time and deadline for each task is: " << endl;
                for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                {
                    // energy part
                    // double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                    // err(i - (lastTaskDoNotNeedOptimize + 1), 0) = hyperPeriod / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                    // barrier function part
                    double responseTime = ResponseTimeWAP(taskSetCurr_, A_Global, P_Global, i, responseTimeInitial(i, 0));
                    cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                    // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                    hpTasks.push_back(taskSetCurr_[i]);
                }
            }
        }
        return err;
    }
};

// ------------------ two convenient functions for ClampComputationTime
double JacobianInClamp(TaskSet &tasks, VectorDynamic &comp, int i)
{
    return 1.0 / tasks[i].period * pow(double(tasks[i].executionTime) / comp(i, 0), 3);
};

// to sort from the biggest to smallest
bool comparePair(const pair<int, double> &p1, const pair<int, double> &p2)
{
    return (p1.second > p2.second);
}
// ------------------
VectorDynamic ClampComputationTime(VectorDynamic comp, TaskSet &tasks, int lastTaskDoNotNeedOptimize,
                                   VectorDynamic &responseTimeInitial,
                                   string roundType = roundTypeInClamp)
{
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
        // VectorDynamic warmStart = ResponseTimeOfTaskSetHard(tasks, comp);

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
        while (objectiveVec.size() > 0)
        {
            int currentIndex = objectiveVec[0].first;

            // try to round up, if success, keep the loop; otherwise, eliminate it and high priority tasks
            // can be speeded up, if necessary, by binary search
            comp(currentIndex, 0) += 1;
            taskDuringOpt[currentIndex].executionTime = comp(currentIndex, 0);

            if (not CheckSchedulability(taskDuringOpt, A_Global, P_Global, responseTimeInitial))
            {
                comp(currentIndex, 0) -= 1;
                taskDuringOpt[currentIndex].executionTime = comp(currentIndex, 0);
                minEliminate = currentIndex;
                for (int i = objectiveVec.size() - 1; i > lastTaskDoNotNeedOptimize; i--)
                {
                    if (objectiveVec[i].first <= minEliminate)
                    {
                        objectiveVec.erase(objectiveVec.begin() + i);
                    }
                }
            }
            else
            {
                ;
            }
            iterationNumber++;
            if (iterationNumber > 100)
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
int FindTaskDoNotNeedOptimize(const TaskSet &tasks, VectorDynamic computationTimeVector, int lastTaskDoNotNeedOptimize,
                              VectorDynamic computationTimeWarmStart, double tolerance = eliminateVariableThreshold)
{
    // update the tasks with the new optimal computationTimeVector
    TaskSet tasksCurr = tasks;
    UpdateTaskSetExecutionTime(tasksCurr, computationTimeVector);
    // cout << "eliminateTol" << eliminateTol << endl;
    int N = tasks.size();
    vector<Task> hpTasks = tasksCurr;
    for (int i = N - 1; i >= 0; i--)
    {
        hpTasks.pop_back();
        tasksCurr[i].executionTime += eliminateTol;
        if (not CheckSchedulability(tasksCurr, A_Global, P_Global, computationTimeWarmStart))
            return i;
        tasksCurr[i].executionTime -= eliminateTol;
    }
    return -1;
}

VectorDynamic UnitOptimization(TaskSet &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic &initialEstimate, VectorDynamic &responseTimeInitial)
{
    int N = tasks.size();
    TASK_NUMBER = N;

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
        if (debugMode == 1)
            params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        if (debugMode == 1)
            params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
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
 * for minimization problem
 */
bool checkConvergenceInterior(double oldY, VectorDynamic oldX, double newY, VectorDynamic newX,
                              double relativeErrorTol, double xTol)
{
    double relDiff = (oldY - newY) / oldY;
    double xDiff = (oldX - newX).norm();
    if (relDiff < relErrorTolIPM || xDiff < relativeErrorTol)
        return true;

    else
        return false;
}

/**
 * tasksDuringOpt's tasks are already updated with latest executionTime;
 * this function performs ipm optimization, i.e., adjusting weight for better performance 
 * at float-point percesion; 
 * 
 * once elimination condition changes, the function returns
 */
VectorDynamic UnitOptimizationIPM(TaskSet &tasksDuringOpt, int lastTaskDoNotNeedOptimize,
                                  VectorDynamic &initialEstimate,
                                  VectorDynamic &responseTimeInitial,
                                  VectorDynamic computationTimeVectorLocalOpt)
{
    int N = tasksDuringOpt.size();
    VectorDynamic initialVar = initialEstimate;
    VectorDynamic variNew = initialVar;
    weightEnergy = minWeightToBegin;
    if (not enableIPM)
    {
        // try
        // {
        variNew = UnitOptimization(tasksDuringOpt, lastTaskDoNotNeedOptimize,
                                   initialVar, responseTimeInitial);
        // }
        // catch (...)
        // {
        //     cout << green << "Catch some error, most probably indetermined Jacobian error" << def << endl;
        //     computationTimeVectorLocalOpt = vectorGlobalOpt;
        //     for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        //         variNew(i - lastTaskDoNotNeedOptimize - 1, 0) = vectorGlobalOpt(i, 0);
        // }
        return variNew;
    }

    double resOld = EstimateEnergyTaskSet(tasksDuringOpt, initialEstimate,
                                          lastTaskDoNotNeedOptimize)
                        .sum() /
                    weightEnergy;
    double resNew = resOld;
    double initialEnergy = resOld;

    // iterations
    int iterationNumIPM = 0;
    do
    {
        // Problem:
        // - batch doesn't work, while opt does?
        if (debugMode == 1)
            cout << "Current weightEnergy " << weightEnergy << endl;
        resOld = resNew;
        initialVar = variNew;
        try
        {
            variNew = UnitOptimization(tasksDuringOpt, lastTaskDoNotNeedOptimize,
                                       initialVar, responseTimeInitial);
        }
        catch (...)
        {
            cout << green << "Catch some error, most probably indetermined Jacobian error" << def << endl;
            computationTimeVectorLocalOpt = vectorGlobalOpt;
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                variNew(i - lastTaskDoNotNeedOptimize - 1, 0) = vectorGlobalOpt(i, 0);
        }

        resNew = EstimateEnergyTaskSet(tasksDuringOpt, variNew, lastTaskDoNotNeedOptimize).sum() /
                 weightEnergy;
        weightEnergy *= weightStep;
        punishmentInBarrier *= weightStep;
        if (debugMode == 1)
            cout << "After one iteration of inside IPM, the current ratio is "
                 << resNew / initialEnergy << " Iteration number is " << iterationNumIPM << endl;
        iterationNumIPM++;
    } while (!checkConvergenceInterior(resOld, initialVar, resNew, variNew, relErrorTolIPM, xTolIPM) &&
             lastTaskDoNotNeedOptimize ==
                 FindTaskDoNotNeedOptimize(tasksDuringOpt,
                                           computationTimeVectorLocalOpt,
                                           lastTaskDoNotNeedOptimize, responseTimeInitial) &&
             iterationNumIPM <= iterationNumIPM_Max);
    return variNew;
}

/**
 * Perform optimization for one task set;
 * this function only performs optimization and elimination, it does not change weights
 **/
double OptimizeTaskSetOneIte(TaskSet &tasks)
{
    int N = tasks.size();
    // vectorGlobalOpt.resize(N, 1);

    // this function also checks schedulability
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard(tasks);
    if (responseTimeInitial(0, 0) == -1)
        return -2;

    VectorDynamic initialExecutionTime = GetParameterVD<int>(tasks, "executionTime");

    int lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(tasks, initialExecutionTime, 0, responseTimeInitial);
    if (lastTaskDoNotNeedOptimize == N - 1)
        return 1;

    // its size is always N
    VectorDynamic computationTimeVectorLocalOpt = initialExecutionTime;
    vectorGlobalOpt = initialExecutionTime;

    int numberOfIteration = 0;
    TaskSet tasksDuringOpt = tasks;

    do
    {
        responseTimeInitial = ResponseTimeOfTaskSetHardWarmStart(tasksDuringOpt, responseTimeInitial);
        if (responseTimeInitial(0, 0) == -1)
        {
            cout << red << "The task becomes unschedulable during optimization" << def << endl;
            throw;
        }

        VectorDynamic initialEstimateDuringOpt;
        initialEstimateDuringOpt.resize(numberOfTasksNeedOptimize, 1);
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            initialEstimateDuringOpt(i - lastTaskDoNotNeedOptimize - 1, 0) =
                computationTimeVectorLocalOpt(i, 0);

        // perform optimization
        VectorDynamic optComp = UnitOptimizationIPM(tasksDuringOpt, lastTaskDoNotNeedOptimize,
                                                    initialEstimateDuringOpt, responseTimeInitial,
                                                    computationTimeVectorLocalOpt);
        // formulate new computationTime
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            computationTimeVectorLocalOpt(i, 0) = optComp(i - lastTaskDoNotNeedOptimize - 1, 0);

        computationTimeVectorLocalOpt = ClampComputationTime(computationTimeVectorLocalOpt,
                                                             tasks, lastTaskDoNotNeedOptimize,
                                                             responseTimeInitial, "rough");
        // cout << computationTimeVectorLocalOpt << endl;
        // find variables to eliminate
        int adjustEliminateTolNum = 0;
        // cout << eliminateTol << endl;
        int lastTaskDoNotNeedOptimizeAfterOpt;
        while (adjustEliminateTolNum < 10)
        {
            lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(
                tasksDuringOpt,
                computationTimeVectorLocalOpt, lastTaskDoNotNeedOptimize, responseTimeInitial);
            if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
                eliminateTol *= eliminateStep;
            adjustEliminateTolNum++;
        }

        if (debugMode == 1)
        {
            cout << "After one iteration, the computationTimeVectorLocalOpt is " << computationTimeVectorLocalOpt << endl;
            cout << "After one iteration, the vectorGlobalOpt is " << vectorGlobalOpt << endl;

            TaskSet tasks2 = tasks;
            for (int i = 0; i < N; i++)
                tasks2[i].executionTime = computationTimeVectorLocalOpt(i, 0);
            VectorDynamic ttt = ResponseTimeOfTaskSetHard(tasks2);
        }

        // check optimization results to see if there are tasks to remove further
        for (int i = lastTaskDoNotNeedOptimize + 1; i <= lastTaskDoNotNeedOptimizeAfterOpt; i++)
        {
            tasksDuringOpt[i].executionTime = computationTimeVectorLocalOpt(i, 0);
        }
        lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;

        numberOfIteration++;
        if (numberOfIteration > N)
        {
            cout << red << "Iteration number error!\n"
                 << def << endl;
            if (debugMode == 1)
                Print(tasks);
            throw;
        }
    } while (numberOfTasksNeedOptimize > 0);

    // performance evaluation
    if (CheckSchedulability(tasksDuringOpt, A_Global, P_Global, responseTimeInitial))
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
        double initialEnergyCost = EstimateEnergyTaskSet(tasks, initialExecutionTime).sum();
        double afterEnergyCost = EstimateEnergyTaskSet(tasks, computationTimeVectorLocalOpt).sum();
        return afterEnergyCost / initialEnergyCost;
    }
    else
    {
        cout << "Unfeasible!" << endl;
        return -1;
    }
    return -1;
}

/**
 * initialize all the global variables
 */
double OptimizeTaskSet(TaskSet &tasks)
{
    int N = tasks.size();
    vectorGlobalOpt.resize(N, 1);
    vectorGlobalOpt.setZero();
    valueGlobalOpt = INT64_MAX;
    weightEnergy = minWeightToBegin;
    A_Global = GenerateZeroMatrix(N);
    P_Global = GenerateZeroMatrix(N);

    double eliminateTolRef = eliminateTol;

    double res = OptimizeTaskSetOneIte(tasks);
    eliminateTol = eliminateTolRef;
    return res;
}