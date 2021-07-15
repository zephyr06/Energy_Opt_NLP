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
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) = 1.0 / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                currentEnergyConsumption += err(i - (lastTaskDoNotNeedOptimize + 1), 0);
                // barrier function part
                double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
                // cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                if (tasks_[i].deadline - responseTime < 0)
                    flagSchedulable = false;
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].executionTime - taskSetCurr_[i].executionTime);
                hpTasks.push_back(taskSetCurr_[i]);
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
                err(i - (lastTaskDoNotNeedOptimize + 1), 0) = 1.0 / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
            }
            return err;
        };

        VectorDynamic err;
        err = f(executionTimeVector);

        if (H)
        {
            *H = NumericalDerivativeDynamicUpper(f, executionTimeVector, deltaOptimizer, numberOfTasksNeedOptimize);
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

            TaskSet taskSetCurr_ = tasks_;
            UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector, lastTaskDoNotNeedOptimize);
            vector<Task> hpTasks;
            for (int i = 0; i < lastTaskDoNotNeedOptimize + 1; i++)
            {
                hpTasks.push_back(taskSetCurr_[i]);
            }
            if (debugMode == 1)
                cout << "The response time and deadline for each task is: " << endl;
            for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            {
                // energy part
                // double frequency = tasks_[i].executionTime / taskSetCurr_[i].executionTime;
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) = hyperPeriod / tasks_[i].period * EstimateEnergyTask(tasks_[i], frequency);
                // barrier function part
                double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeInitial(i, 0), taskSetCurr_[i], hpTasks);
                if (debugMode == 1)
                    cout << responseTime << ", " << taskSetCurr_[i].deadline << endl;
                // err(i - (lastTaskDoNotNeedOptimize + 1), 0) += Barrier(tasks_[i].deadline - responseTime);
                hpTasks.push_back(taskSetCurr_[i]);
            }
        }
        return err;
    }
};

void ClampComputationTime(VectorDynamic &comp)
{
    // int n = comp.rows();
    // for (int i = 0; i < n; i++)
    //     comp(i, 0) = int(comp(i, 0));
    return;
}

/**
 * find the tasks that do not need to optimize;
 * i means i-th task do not need optimization,  while i+1, ..., N need
 * -1 means all tasks need optimization
 * N-1 means all tasks do not need optimization
 **/
int FindTaskDoNotNeedOptimize(const TaskSet &tasks, VectorDynamic computationTimeVector, int endSearchAt,
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
        double rt = ResponseTimeAnalysisWarm(computationTimeWarmStart(i, 0), tasksCurr[i], hpTasks);
        if (abs(rt - tasks[i].deadline) <= tolerance || rt > tasks[i].deadline)
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
    auto model = noiseModel::Isotropic::Sigma(numberOfTasksNeedOptimize, noiseModelSigma);
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

bool checkConvergenceInterior(double oldRes, double newRes)
{
    double relDiff = (oldRes - newRes) / oldRes;
    if (relDiff < -1e-5)
    {
        cout << red << "After one iteration, performance drops!" << def << endl;
        return true;
    }
    else if (relDiff < relErrorTolIPM)
        return true;

    else
        return false;
}

/**
 * tasksDuringOpt's tasks are already updated with latest executionTime
 */
VectorDynamic UnitOptimizationIPM(TaskSet &tasksDuringOpt, int lastTaskDoNotNeedOptimize, VectorDynamic &initialEstimate, VectorDynamic &responseTimeInitial)
{
    if (not enableIPM)
    {
        return UnitOptimization(tasksDuringOpt, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);
    }

    int N = tasksDuringOpt.size();
    VectorDynamic dummy;
    dummy.resize(1, 1);
    dummy.setZero();
    double oldRes = 0;
    for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        oldRes += 1.0 / tasksDuringOpt[i].period *
                  EstimateEnergyTask(tasksDuringOpt[i], 1.0);
    double newRes = oldRes;
    double initialEnergy = oldRes / weightEnergy;

    // iterations

    VectorDynamic initial;
    initial.resize(N, 1);
    initial.setZero();
    weightEnergy = minWeightToBegin;
    if (tasksDuringOpt[19].deadline == 9280 && tasksDuringOpt[18].deadline == 8180)
    {
        int a = 1;
    }
    do
    {
        // Problem:
        // - UnitOptimization's initial has not been updated
        // - lastTaskDoNotNeedOptimize has not been updated after optimization
        // - batch doesn't work, while opt does
        cout << "Current weightEnergy " << weightEnergy << endl;
        oldRes = newRes;
        VectorDynamic variNew = UnitOptimization(tasksDuringOpt, lastTaskDoNotNeedOptimize,
                                                 initialEstimate, responseTimeInitial);
        initial = variNew;
        ClampComputationTime(variNew);

        newRes = 0;
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            newRes += 1.0 / tasksDuringOpt[i].period *
                      EstimateEnergyTask(tasksDuringOpt[i],
                                         tasksDuringOpt[i].executionTime / variNew(i - lastTaskDoNotNeedOptimize - 1, 0)) /
                      weightEnergy;
        weightEnergy *= weightStep;
        punishmentInBarrier *= weightStep;

        cout << "After one iteration of inside IPM, the current ratio is " << newRes / initialEnergy << endl;
        // cout << "The computationTimeVector is " << initial << endl;
        lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(tasksDuringOpt, initialExecutionTime, 0, responseTimeInitial)
    } while (not checkConvergenceInterior(oldRes, newRes) || lastTaskDoNotNeedOptimize == N - 1);
    return initial;
}

/**
 * Perform optimization for one task set
 **/
pair<double, VectorDynamic> OptimizeTaskSetOneIte(TaskSet &tasks, VectorDynamic &initial)
{
    int N = tasks.size();
    // vectorGlobalOpt.resize(N, 1);

    VectorDynamic dummy;
    dummy.resize(1, 1);
    dummy.setZero();
    // this function also checks schedulability
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSetHard(tasks);
    if (responseTimeInitial(0, 0) == -1)
        return make_pair(-2, dummy);
    VectorDynamic initialExecutionTime;
    if (initial.sum() != 0)
    {
        if (initial.rows() == N)
            initialExecutionTime = initial;
        else
        {
            cout << red << "Input parameter error!" << def << endl;
            throw;
        }
    }
    else
    {
        initialExecutionTime.resize(N, 1);
        for (int i = 0; i < N; i++)
            initialExecutionTime(i, 0) = tasks[i].executionTime;
    }
    // cout<<"initialExecutionTime "<<initialExecutionTime<<endl;
    int lastTaskDoNotNeedOptimize = FindTaskDoNotNeedOptimize(tasks, initialExecutionTime, 0, responseTimeInitial);
    // int numberOfTasksNeedOptimize = N - (lastTaskDoNotNeedOptimize + 1);
    if (lastTaskDoNotNeedOptimize == N - 1)
        return make_pair(1.0, dummy);

    bool stop = false;
    VectorDynamic optComp;
    VectorDynamic computationTimeVectorLocalOpt = initialExecutionTime;
    if (vectorGlobalOpt.sum() == 0)
        vectorGlobalOpt = initialExecutionTime;

    int numberOfIteration = 0;
    TaskSet tasksDuringOpt = tasks;
    const double weightEnergyRef = weightEnergy;

    // //for debug
    // if (tasks[0].period == 120 && tasks[1].period == 170 && tasks[0].deadline == 48)
    //     int a = 1;

    while (not stop && numberOfTasksNeedOptimize > 0)
    {
        VectorDynamic initialEstimate;
        initialEstimate.resize(numberOfTasksNeedOptimize, 1);
        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            initialEstimate(i - lastTaskDoNotNeedOptimize - 1, 0) = computationTimeVectorLocalOpt(i, 0);

        // perform optimization

        for (int i = 0; i < weightEnergyMaxOrder; i++)
        {
            weightEnergy = weightEnergyRef * pow(10, -1 * i);
            if (debugMode == 1)
                cout << "Current weightEnergy is " << weightEnergy << endl;
            try
            {
                optComp = UnitOptimizationIPM(tasksDuringOpt, lastTaskDoNotNeedOptimize, initialEstimate, responseTimeInitial);
                // formulate new computationTime
                for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
                    computationTimeVectorLocalOpt(i, 0) = optComp(i - lastTaskDoNotNeedOptimize - 1, 0);

                weightEnergy = weightEnergyRef;
            }
            catch (...)
            {
                cout << green << "Catch some error, most probably indetermined Jacobian error" << def << endl;
                computationTimeVectorLocalOpt = vectorGlobalOpt;
            }
        }

        ClampComputationTime(computationTimeVectorLocalOpt);

        // find variables to eliminate
        int lastTaskDoNotNeedOptimizeAfterOpt = FindTaskDoNotNeedOptimize(tasksDuringOpt,
                                                                          computationTimeVectorLocalOpt, lastTaskDoNotNeedOptimize, responseTimeInitial);

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

        if (lastTaskDoNotNeedOptimizeAfterOpt != N - 1 && lastTaskDoNotNeedOptimizeAfterOpt == lastTaskDoNotNeedOptimize)
        {
            // weightEnergy *= weightStep;
            // eliminateTol /= 10;
            // stop = true;
            // this will cause iteration number error next!
            stop = true;
        }
        else if (lastTaskDoNotNeedOptimizeAfterOpt == N - 1)
            stop = true;
        else
        {
            lastTaskDoNotNeedOptimize = lastTaskDoNotNeedOptimizeAfterOpt;
            for (int i = 0; i <= lastTaskDoNotNeedOptimize; i++)
            {
                tasksDuringOpt[i].executionTime = computationTimeVectorLocalOpt(i, 0);
            }
        }

        numberOfIteration++;
        if (numberOfIteration > N)
        {
            cout << red << "Iteration number error!\n"
                 << def << endl;
            if (debugMode == 1)
                Print(tasks);
            throw;
        }
    }

    TaskSet tasks2 = tasks;
    UpdateTaskSetExecutionTime(tasks2, computationTimeVectorLocalOpt);
    bool a = CheckSchedulability<int>(tasks2);
    if (a)
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

        VectorDynamic initialExecutionTime0;
        initialExecutionTime0.resize(N, 1);
        for (int i = 0; i < N; i++)
            initialExecutionTime0(i, 0) = tasks[i].executionTime;
        double initialEnergyCost = EstimateEnergyTaskSet(tasks, initialExecutionTime0).sum();
        double afterEnergyCost = EstimateEnergyTaskSet(tasks, computationTimeVectorLocalOpt).sum();

        return make_pair(afterEnergyCost / initialEnergyCost, computationTimeVectorLocalOpt);
    }
    else
    {
        // TODO: in this case, add more loops to try different values of weightEnergy
        cout << "Unfeasible!" << endl;

        return make_pair(-1, dummy);
    }
    return make_pair(0, dummy);
}

double OptimizeTaskSet(TaskSet &tasks)
{
    int N = tasks.size();
    vectorGlobalOpt.resize(N, 1);
    vectorGlobalOpt.setZero();
    valueGlobalOpt = INT64_MAX;

    // iterations
    // double oldRes = 1.0, newRes = 1.0;
    VectorDynamic initial;
    initial.resize(N, 1);
    initial.setZero();
    weightEnergy = minWeightToBegin;
    // do
    // {
    //     oldRes = newRes;
    //     auto res = OptimizeTaskSetOneIte(tasks, initial);
    //     newRes = res.first;
    //     initial = res.second;
    //     weightEnergy *= weightStep;
    //     eliminateTol /= eliminateStep;
    //     cout << "After one iteration of IPM, the current ratio is " << newRes << endl;
    //     cout << "The computationTimeVector is " << initial << endl;
    // } while (not checkConvergenceInterior(oldRes, newRes));

    return OptimizeTaskSetOneIte(tasks, initial).first;
}