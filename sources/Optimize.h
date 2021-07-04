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

using namespace std;
using namespace gtsam;

#define numberOfTasksNeedOptimize (N - lastTaskDoNotNeedOptimize - 1)

/**
 * barrier function for the optimization
 **/
double Barrier(double x)
{
    if (x > 0)
        // return pow(x, 2);
        return weightLogBarrier * log(x);
    else if (x < 0)
        return punishmentInBarrier * pow(10, TASK_NUMBER - 3) * pow(1 - x, 1);
    else // it basically means x=0
        return weightLogBarrier * log(x + toleranceBarrier);
}

void UpdateTaskSetExecutionTime(TaskSet &taskSet, VectorDynamic executionTimeVec, int lastTaskDoNotNeedOptimize = -1)
{
    int N = taskSet.size();

    for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        taskSet[i].executionTime = executionTimeVec(i - lastTaskDoNotNeedOptimize - 1, 0);
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

        // cout << "resMinus" << endl
        //      << resMinus << endl;
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
        // cout << "resPlus" << endl
        //      << resPlus << endl;
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
    long long int hyperPeriod;
    ComputationTimeVector responseTimeBase_;

    ComputationFactor(Key key, TaskSet &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic responseTimeInitial,
                      SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                tasks_(tasks), lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
                                                responseTimeInitial(responseTimeInitial)
    {
        N = tasks_.size();
        // numberOfTasksNeedOptimize = N - lastTaskDoNotNeedOptimize - 1;
        hyperPeriod = HyperPeriod(tasks_);
        cout << "The hypeprperiod is " << hyperPeriod << endl;
    }

    Vector evaluateError(const VectorDynamic &executionTimeVector, boost::optional<Matrix &> H = boost::none) const override
    {
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            VectorDynamic err;
            err.resize(numberOfTasksNeedOptimize, 1);
            TaskSet taskSetCurr_ = tasks_;
            UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);

            vector<Task> hpTasks;
            for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
            {
                hpTasks.push_back(taskSetCurr_[i]);
            }
            // cout << "The response time and deadline for each task is: " << endl;
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            {
                // energy part
                double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) = hyperPeriod / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                // barrier function part
                double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
                // cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].executionTime - taskSetCurr_[i].executionTime);
                hpTasks.push_back(taskSetCurr_[i]);
            }
            return err;
        };
        boost::function<Matrix(const VectorDynamic &)> f2 =
            [this](const VectorDynamic &executionTimeVector)
        {
            VectorDynamic err;
            err.resize(numberOfTasksNeedOptimize, 1);
            TaskSet taskSetCurr_ = tasks_;
            UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);

            vector<Task> hpTasks;
            for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
            {
                hpTasks.push_back(taskSetCurr_[i]);
            }
            // cout << "The response time and deadline for each task is: " << endl;
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            {
                // energy part
                double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) = hyperPeriod / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                // barrier function part
                // double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
                // cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                hpTasks.push_back(taskSetCurr_[i]);
            }
            return err;
        };

        VectorDynamic err;
        err = f(executionTimeVector);
        // err.resize(numberOfTasksNeedOptimize, 1);
        // // JacobianOpt jacobian;
        // // jacobian.setZero();
        // TaskSet taskSetCurr_ = tasks_;
        // UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);

        // cout << "The hyper period is " << hyperPeriod << endl;

        // vector<Task> hpTasks;
        // for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
        // {
        //     hpTasks.push_back(taskSetCurr_[i]);
        // }
        // cout << "The response time and deadline for each task is: " << endl;
        // for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        // {
        //     if (taskSetCurr_[i].executionTime > 33 && taskSetCurr_[i].executionTime < 34)
        //     {
        //         int a = 1;
        //     }
        //     // energy part
        //     double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
        //     err(i - (lastTaskDoNotNeedOptimize + 1), 0) = hyperPeriod / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
        //     // barrier function part
        //     double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
        //     cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
        //     err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
        //     hpTasks.push_back(taskSetCurr_[i]);
        // }
        // cout << endl;

        if (H)
        {
            // *H = NumericalDerivativeDynamicUpper(f, executionTimeVector, deltaOptimizer, numberOfTasksNeedOptimize);
            *H = NumericalDerivativeDynamic(f2, executionTimeVector, deltaOptimizer, numberOfTasksNeedOptimize);
            // *H = jacobian;
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
                double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
                cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                hpTasks.push_back(taskSetCurr_[i]);
            }
        }
        // cout << "The current evaluation point is " << endl
        //      << executionTimeVector << endl
        //      << endl;
        // cout << "The current error is " << endl
        //      << err << endl
        //      << err.norm() << endl
        //      << endl;
        return err;
    }
};

// VectorDynamic InitializeOptimization(const TaskSet &tasks, int numberOfTasksNeedOptimize)
// {
//     VectorDynamic comp;
//     // TODO: need test
//     // comp.resize(numberOfTasksNeedOptimize, 1);
//     // for (int i = 0; i < numberOfTasksNeedOptimize; i++)
//     //     comp(i, 0) = tasks[i].executionTime;
//     return comp;
// }

void ClampComputationTime(VectorDynamic &comp)
{
    int n = comp.rows();
    for (int i = 0; i < n; i++)
        comp(i, 0) = int(comp(i, 0));
}

/**
 * find the tasks that do not need to optimize;
 * i means i-th task do not need optimization,  while i+1, ..., N need
 * -1 means all tasks need optimization
 * N-1 means all tasks do not need optimization
 **/
int FindTaskDoNotNeedOptimize(const TaskSet &tasks, VectorDynamic computationTimeVector, int endSearchAt,
                              VectorDynamic computationTimeWarmStart, double tolerance = toleranceInOuterLoop)
{
    // update the tasks with the new optimal computationTimeVector
    TaskSet tasksCurr = tasks;
    UpdateTaskSetExecutionTime(tasksCurr, computationTimeVector);

    int N = tasks.size();
    vector<Task> hpTasks = tasks;
    for (int i = N - 1; i >= 0; i--)
    {
        hpTasks.pop_back();
        double rt = ResponseTimeAnalysisWarm(computationTimeWarmStart(i, 0), tasksCurr[i], hpTasks);
        if (abs(rt - tasks[i].deadline) <= tolerance)
            return i;
    }
    return -1;
}

VectorDynamic UnitOptimization(TaskSet &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic &initialEstimate, VectorDynamic &responseTimeInitial)
{
    int N = tasks.size();
    // int numberOfTasksNeedOptimize = N - lastTaskDoNotNeedOptimize - 1;

    // build the factor graph
    auto model = noiseModel::Isotropic::Sigma(numberOfTasksNeedOptimize, noiseModelSigma);
    NonlinearFactorGraph graph;
    Symbol key('a', 0);
    graph.emplace_shared<ComputationFactor>(key, tasks, lastTaskDoNotNeedOptimize, responseTimeInitial, model);

    // ComputationTimeVector comp;
    // comp << 10, 20, 30;
    Values initialEstimateFG;
    initialEstimateFG.insert(key, initialEstimate);

    // usually, when the change of variables between steps is smaller than 1,
    // we can already terminate; the corresponding minimal of relative error is
    // approximately 2e-3;

    Values result;
    if (optimizerType == 1)
    {
        DoglegParams params;
        params.setVerbosityDL("DELTA");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp = result.at<VectorDynamic>(key);
    cout << "After optimization, the computation time vector is " << optComp << endl;
    ClampComputationTime(optComp);
    cout << "After clamp, the computation time vector is " << optComp << endl;
    return optComp;
}

/**
 * Perform optimization for one task set
 **/
double OptimizeTaskSetOneIte(TaskSet &tasks)
{
    int N = tasks.size();

    // this function also checks schedulability
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard(tasks);
    if (responseTimeInitial(0, 0) == -1)
        return -2;
    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;
    int lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(tasks, initialExecutionTime, 0, responseTimeInitial);
    // int numberOfTasksNeedOptimize = N - (lastTaskDoNotNeedOptimize + 1);

    bool stop = false;
    VectorDynamic optComp;
    VectorDynamic computationTimeVector = initialExecutionTime;

    int numberOfIteration = 0;
    TaskSet tasksDuringOpt = tasks;
    while (not stop)
    {
        VectorDynamic initialEstimate;
        initialEstimate.resize(numberOfTasksNeedOptimize, 1);
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            initialEstimate(i - lastTaskDoNotNeedOptimize - 1, 0) = computationTimeVector(i, 0);

        // perform optimization
        optComp = UnitOptimization(tasksDuringOpt, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);

        // formulate new computationTime
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            computationTimeVector(i, 0) = optComp(i - lastTaskDoNotNeedOptimize - 1, 0);

        cout << "After one iteration, the computation time is " << computationTimeVector << endl;

        // check optimization results to see if there are tasks to remove further
        int lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(tasksDuringOpt, computationTimeVector, lastTaskDoNotNeedOptimize, responseTimeInitial);
        if (lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize || lastTaskDoNotNeedOptimizeAfterOpt == N - 1)
            stop = true;
        else
        {
            lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;
            for (int i = 0; i <= lastTaskDoNotNeedOptimize; i++)
            {
                tasksDuringOpt[i].executionTime = computationTimeVector(i, 0);
            }
        }

        numberOfIteration++;
        if (numberOfIteration > N)
        {
            cout << "Iteration number error!\n";
            throw;
        }
    }

    TaskSet tasks2 = tasks;
    UpdateTaskSetExecutionTime(tasks2, computationTimeVector);
    bool a = CheckSchedulability<int>(tasks2);
    if (a)
    {
        cout << "The task set is schedulable after optimization\n";
        cout << endl;
        cout << "The original task set is: " << endl;
        for (int i = 0; i < N; i++)
        {
            cout << i << " ";
            tasks[i].print();
        }
        double initialEnergyCost = EstimateEnergyTaskSet(tasks, initialExecutionTime).sum();
        double afterEnergyCost = EstimateEnergyTaskSet(tasks, computationTimeVector).sum();

        return afterEnergyCost / initialEnergyCost;
    }
    else
    {
        // TODO: in this case, add more loops to try different values of weightEnergy
        cout << "Unfeasible!" << endl;
        return -1;
    }
    return 0;
}

double OptimizeTaskSet(TaskSet &tasks)
{
    double weightEnergyRef = weightEnergy;

    long long int hyperPeriod = HyperPeriod(tasks);

    // adjust some parameters based on scale
    weightEnergy = weightEnergy / log10(hyperPeriod);

    for (int i = 0; i < weightEnergyMaxOrder; i++)
    {
        weightEnergy = weightEnergyRef * pow(10, i);
        cout << "Current weightEnergy is " << weightEnergy << endl;
        try
        {
            double res = OptimizeTaskSetOneIte(tasks);
            if (res != -1)
            {
                weightEnergy = weightEnergyRef;
                return res;
            }
        }
        catch (...)
        {
            ;
        }

        weightEnergy = weightEnergyRef * pow(10, -1 * i);
        cout << "Current weightEnergy is " << weightEnergy << endl;
        try
        {
            double res = OptimizeTaskSetOneIte(tasks);
            if (res != -1)
            {
                weightEnergy = weightEnergyRef;
                return res;
            }
        }
        catch (...)
        {
            ;
        }
    }

    weightEnergy = weightEnergyRef;
    return -1;
}