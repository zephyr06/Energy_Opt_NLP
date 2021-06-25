#pragma once

#include <chrono>
#include <math.h>

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

/**
 * barrier function for the optimization
 **/
float Barrier(float x)
{
    if (x > 0)
        // return pow(x, 2);
        return weightLogBarrier * log(x);
    else if (x < 0)
        return punishmentInBarrier * pow(10, TASK_NUMBER - 3) * pow(1 - x, 1);
    else // it basically means x=0
        return weightLogBarrier * log(x + toleranceBarrier);
}

void UpdateTaskSetExecutionTime(TaskSet &taskSet, ComputationTimeVector executionTimeVec)
{
    int i = 0;
    for (auto &task : taskSet)
        task.executionTime = executionTimeVec(i++, 0);
}

class ComputationFactor : public NoiseModelFactor1<ComputationTimeVector>
{
public:
    TaskSet tasks_;
    ComputationTimeVector responseTimeBase_;

    ComputationFactor(Key key, TaskSet &tasks,
                      SharedNoiseModel model) : NoiseModelFactor1<ComputationTimeVector>(model, key),
                                                tasks_(tasks)
    {
        for (int i = 0; i < TASK_NUMBER; i++)
        {
            vector<Task> hpTasks;
            for (int j = 0; j < i; j++)
                hpTasks.push_back(tasks_[j]);

            responseTimeBase_[i] = ResponseTimeAnalysis<float>(tasks_[i], hpTasks);
        }
    }

    Vector evaluateError(const ComputationTimeVector &executionTimeVector, boost::optional<Matrix &> H = boost::none) const override
    {
        ErrElement err;
        JacobianOpt jacobian;
        jacobian.setZero();
        TaskSet taskSetCurr_ = tasks_;
        UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector);

        long long int hyperPeriod = 0;
        if (H)
            hyperPeriod = HyperPeriod(tasks_);

        err = EstimateEnergyTaskSet(tasks_, executionTimeVector);
        // todo: find a way to test the evaluteError function
        for (int i = 0; i < TASK_NUMBER; i++)
        {
            Task taskCurr_(tasks_[i]);

            taskCurr_.executionTime = executionTimeVector(i, 0);
            vector<Task> hpTasks;
            for (int j = 0; j < i; j++)
                hpTasks.push_back(taskSetCurr_[j]);

            float responseTime = ResponseTimeAnalysisWarm<float>(responseTimeBase_[i], taskCurr_, hpTasks);

            err(i, 0) += Barrier(tasks_[i].deadline - responseTime);
            // approximate the Jacobian
            if (H)
            {
                if (tasks_[i].deadline - responseTime > 0)
                {
                    jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
                    jacobian(i, i) -= 1 / (tasks_[i].deadline - responseTime);
                    for (int j = 0; j < i; j++)
                    {
                        jacobian(i, j) = -1 / (tasks_[i].deadline - responseTime) * ceil(responseTime / tasks_[j].period);
                    }
                }
                else if (tasks_[i].deadline - responseTime <= 0)
                {
                    jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
                    jacobian(i, i) += punishmentInBarrier * pow(10, TASK_NUMBER - 3) * 1;
                    for (int j = 0; j < i; j++)
                    {
                        jacobian(i, j) = punishmentInBarrier * pow(10, TASK_NUMBER - 3) * ceil(responseTime / tasks_[j].period);
                    }
                }
                else // tasks_[i].deadline - responseTime == 0
                {
                    jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
                    jacobian(i, i) -= 1 / (tasks_[i].deadline - responseTime + toleranceBarrier);
                    for (int j = 0; j < i; j++)
                    {
                        jacobian(i, j) = -1 / (tasks_[i].deadline - responseTime + toleranceBarrier) * ceil(responseTime / tasks_[j].period);
                    }
                }
            }
        }

        if (H)
        {
            boost::function<Matrix(const ComputationTimeVector &)> f =
                [this](const ComputationTimeVector &input)
            {
                ErrElement err;
                TaskSet taskSetCurr_ = tasks_;
                UpdateTaskSetExecutionTime(taskSetCurr_, input);

                // todo: find a way to test the evaluteError function
                err = EstimateEnergyTaskSet(tasks_, input);
                // todo: find a way to test the evaluteError function
                for (int i = 0; i < TASK_NUMBER; i++)
                {
                    Task taskCurr_(tasks_[i]);

                    taskCurr_.executionTime = input(i, 0);
                    vector<Task> hpTasks;
                    for (int j = 0; j < i; j++)
                        hpTasks.push_back(taskSetCurr_[j]);

                    float responseTime = ResponseTimeAnalysisWarm<float>(responseTimeBase_[i], taskCurr_, hpTasks);

                    float tt = Barrier(tasks_[i].deadline - responseTime);
                    err(i, 0) += tt;
                }
                return err;
            };

            // TWO things to notice there
            // - When the iteration points are close to the barrier function bounder,
            // the first order numerical derivative will be unaccurate because of the log function,
            // and so gives different results than analytic Jacobian;

            // - At the very close boundary where numerical Jacobian may give a
            // very large result because of the punishment coefficient, it may be okay to use either form;
            // The log barrier function makes the Jacobian very large already,
            // and so the update step should be small correspondingly

            *H = numericalDerivative11(f, executionTimeVector,
                                       deltaOptimizer);
            // *H = jacobian;

            cout << "The current evaluation point is " << endl
                 << executionTimeVector << endl;
            cout << "The Jacobian is " << endl
                 << *H << endl;
            cout << "The approximated Jacobian is " << endl
                 << jacobian << endl;
            cout << "The current error is " << endl
                 << err << endl
                 << err.norm() << endl
                 << endl;
        }
        return err;
    }
};

ComputationTimeVector InitializeOptimization(const TaskSet &tasks)
{
    ComputationTimeVector comp;
    for (int i = 0; i < TASK_NUMBER; i++)
        comp(i, 0) = tasks[i].executionTime;
    return comp;
}

void ClampComputationTime(ComputationTimeVector &comp)
{
    for (int i = 0; i < TASK_NUMBER; i++)
        comp(i, 0) = int(comp(i, 0));
}

/**
 * Perform optimization for one task set
 **/
float OptimizeTaskSet(TaskSet &tasks)
{
    // test schedulability
    if (!CheckSchedulability<int>(tasks))
    {
        cout << "The task set is not schedulable!\n";
        return -1;
    }

    // build the factor graph since there

    auto model = noiseModel::Isotropic::Sigma(TASK_NUMBER, noiseModelSigma);

    NonlinearFactorGraph graph;
    Symbol key('a', 0);

    graph.emplace_shared<ComputationFactor>(key, tasks, model);

    // ComputationTimeVector comp;
    // comp << 10, 20, 30;
    Values initialEstimate;
    initialEstimate.insert(key, InitializeOptimization(tasks));

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
        DoglegOptimizer optimizer(graph, initialEstimate, params);
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
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
        result = optimizer.optimize();
    }

    ComputationTimeVector optComp = result.at<ComputationTimeVector>(key);
    cout << "After optimization, the computation time vector is " << optComp << endl;
    ClampComputationTime(optComp);

    TaskSet tasks2 = tasks;
    UpdateTaskSetExecutionTime(tasks2, optComp);
    bool a = CheckSchedulability<int>(tasks2);
    if (a)
    {
        cout << "The task set is schedulable after optimization\n";
        double initialEnergyCost = EstimateEnergyTaskSet(tasks, InitializeOptimization(tasks)).sum();
        double afterEnergyCost = EstimateEnergyTaskSet(tasks, optComp).sum();
        return afterEnergyCost / initialEnergyCost;
    }
    else
    {
        cout << "Unfeasible!" << endl;
        return -1;
    }
}
