
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/ReadControlCases.h"
#include "../sources/CoeffFactor.h"
#include "../sources/RTAFactor.h"
#include "../sources/InequalifyFactor.h"

#include "ControlFactorGraphUtils.h"
#include "FactorGraphForceManifold.h"
#include "FactorGraphInManifold.h"

template <typename FactorGraphType>
pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic &coeff,
                                                   std::vector<bool> &maskForElimination)
{
    BeginTimer(__func__);
    NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(maskForElimination, tasks, coeff);
    if (debugMode == 1)
    {
        graph.print();
    }
    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    Values initialEstimateFG = FactorGraphType::GenerateInitialFG(tasks, maskForElimination);
    if (debugMode == 1)
    {
        cout << Color::green;
        // std::lock_guard<std::mutex> lock(mtx);
        auto sth = graph.linearize(initialEstimateFG)->jacobian();
        MatrixDynamic jacobianCurr = sth.first;
        std::cout << "Current Jacobian matrix:" << endl;
        std::cout << jacobianCurr << endl;
        std::cout << "Current b vector: " << endl;
        std::cout << sth.second << endl;
        cout << Color::def << endl;
    }

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
        params.setVerbosityLM(verbosityLM);
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setLinearSolverType(linearOptimizerType);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp, rtaFromOpt; // rtaFromOpt can only be used for 'cout'
    optComp = FactorGraphType::ExtractResults(result, tasks);
    UpdateTaskSetPeriod(tasks, optComp);
    rtaFromOpt = RTALLVector(tasks);
    if (debugMode == 1)
    {
        cout << endl;
        cout << Color::blue;
        cout << "After optimization, the period vector is " << endl
             << optComp << endl;
        cout << "After optimization, the rta vector is " << endl
             << rtaFromOpt << endl;
        cout << "The graph error is " << graph.error(result) << endl;
        cout << Color::def;
        cout << endl;
        cout << Color::blue;
        UpdateTaskSetPeriod(tasks, FactorGraphType::ExtractResults(initialEstimateFG, tasks));
        cout << "Before optimization, the total error is " << RealObj(tasks, coeff) << endl;
        UpdateTaskSetPeriod(tasks, optComp);
        cout << "The objective function is " << RealObj(tasks, coeff) << endl;
        cout << Color::def;
    }

    UpdateTaskSetPeriod(tasks, optComp);
    EndTimer(__func__);
    return make_pair(optComp, RealObj(tasks, coeff));
}

template <typename FactorGraphType>
pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(TaskSet &tasks, VectorDynamic &coeff,
                                                           std::vector<bool> &maskForElimination)
{
    RTA_LL rr(tasks);
    if (!rr.CheckSchedulability(debugMode == 1))
    {
        cout << "The task set is not schedulable!" << endl;
        return make_pair(GetParameterVD<double>(tasks, "period"), 1e30);
    }
    VectorDynamic periodRes;
    double err;
    for (double weight = weightSchedulabilityMin; weight <= weightSchedulabilityMax;
         weight *= weightSchedulabilityStep)
    {
        weightSchedulability = weight;
        std::tie(periodRes, err) = UnitOptimizationPeriod<FactorGraphType>(tasks, coeff, maskForElimination);
        VectorDynamic periodPrev = GetParameterVD<double>(tasks, "period");
        UpdateTaskSetPeriod(tasks, periodRes);
        RTA_LL r(tasks);
        if (!r.CheckSchedulability())
        {
            UpdateTaskSetPeriod(tasks, periodPrev);
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods become unschedulable and are"
                     << endl
                     << periodRes << endl;
                cout << Color::def;
            }

            return make_pair(periodPrev, err);
        }
        else
        {
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods remain schedulable and are"
                     << endl
                     << periodRes << endl;
                cout << Color::def;
            }
        }
    }

    return make_pair(periodRes, err);
}
/* @brief return a string with expected precision by adding leading 0 */
string to_string_precision(int a, int precision)
{
    return std::string(4 - min(4, to_string(a).length()), '0') + to_string(a);
}
template <typename T>
void print(const std::vector<T> &vec)
{
    for (auto x : vec)
    {
        cout << x << ", ";
    }
}

/**
 * @brief round period into int type, we assume the given task set is schedulable!
 *
 * @param tasks
 * @param maskForElimination
 * @param coeff
 */
void RoundPeriod(TaskSet &tasks, std::vector<bool> &maskForElimination, VectorDynamic &coeff)
{
    if (roundTypeInClamp == "none")
        return;
    else if (roundTypeInClamp == "rough")
    {
        for (uint i = 0; i < maskForElimination.size(); i++)
        {
            if (maskForElimination[i])
            {
                tasks[i].period = ceil(tasks[i].period);
            }
        }
    }
    else if (roundTypeInClamp == "fine")
    {
        int N = tasks.size();

        vector<int> wait_for_eliminate_index;
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (maskForElimination[i] && tasks[i].period != ceil(tasks[i].period))
                wait_for_eliminate_index.push_back(i);
        }
        if (!wait_for_eliminate_index.empty())
        {

            VectorDynamic rtaBase = RTALLVector(tasks);

            vector<pair<int, double>> objectiveVec;
            objectiveVec.reserve(wait_for_eliminate_index.size());
            for (uint i = 0; i < wait_for_eliminate_index.size(); i++)
            {
                objectiveVec.push_back(make_pair(wait_for_eliminate_index[i], coeff(wait_for_eliminate_index[i] * 2) * -1));
            }
            sort(objectiveVec.begin(), objectiveVec.end(), comparePair);
            int iterationNumber = 0;

            // int left = 0, right = 0;
            while (objectiveVec.size() > 0)
            {
                int currentIndex = objectiveVec[0].first;

                // try to round 'up', if success, keep the loop; otherwise, eliminate it and high priority tasks
                // can be speeded up, if necessary, by binary search
                int left = int(tasks[currentIndex].period);
                // int left = rtaBase(currentIndex);
                int right = ceil(tasks[currentIndex].period);
                if (left > right)
                {
                    CoutError("left > right error in clamp!");
                }
                int rightOrg = right;
                bool schedulale_flag;
                while (left < right)
                {
                    int mid = (left + right) / 2;

                    tasks[currentIndex].period = mid;
                    RTA_LL r(tasks);
                    schedulale_flag = r.CheckSchedulability(
                        rtaBase, debugMode == 1);
                    if (not schedulale_flag)
                    {
                        left = mid + 1;
                        tasks[currentIndex].period = rightOrg;
                    }
                    else
                    {
                        tasks[currentIndex].period = mid;
                        right = mid;
                    }
                }

                // post processing, left=right is the value we want
                tasks[currentIndex].period = left;
                objectiveVec.erase(objectiveVec.begin() + 0);

                iterationNumber++;
                if (iterationNumber > N)
                {
                    CoutWarning("iterationNumber error in Clamp!");
                    break;
                }
            };
        }
    }
    else
    {
        cout << "input error in ClampComputationTime: " << roundTypeInClamp << endl;
        throw;
    }
    return;
}
template <typename T>
bool EqualVector(std::vector<T> &v1, std::vector<T> &v2)
{
    if (v1.size() != v2.size())
    {
        return false;
    }
    for (uint i = 0; i < v1.size(); i++)
    {
        if (v1[i] != v2[i])
            return false;
    }
    return true;
}
// TODO: limit the number of outer loops
template <typename FactorGraphType>
pair<VectorDynamic, double> OptimizeTaskSetIterative(TaskSet &tasks, VectorDynamic &coeff,
                                                     std::vector<bool> &maskForElimination)
{

    VectorDynamic periodResCurr, periodResPrev;
    std::vector<bool> maskForEliminationPrev = maskForElimination;
    double errPrev = 1e30;
    double errCurr = RealObj(tasks, coeff);
    int loopCount = 0;
    if (enableReorder > -1)
    {
        errCurr = RealObj(tasks, coeff);
        TaskSet tasksTry = tasks;
        VectorDynamic coeffTry = coeff;
        Reorder(tasksTry, coeffTry, "RM");
        double errCurrTry = RealObj(tasksTry, coeffTry);
        if (errCurrTry < errCurr)
        {
            tasks = tasksTry;
            coeff = coeffTry;
        }
    }
    double disturbIte = eliminateTol;
    while (errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop) && ContainFalse(maskForElimination) && loopCount < MaxLoopControl)
    {
        // store prev result
        errPrev = errCurr;
        periodResPrev = GetParameterVD<double>(tasks, "period");
        maskForEliminationPrev = maskForElimination;

        // perform optimization
        double err;
        std::tie(periodResCurr, err) = OptimizeTaskSetIterativeWeight<FactorGraphType>(tasks, coeff, maskForElimination);
        UpdateTaskSetPeriod(tasks, periodResCurr);

        // adjust tasks' priority based on RM
        if (enableReorder > 0)
        {
            errCurr = RealObj(tasks, coeff);
            TaskSet tasksTry = tasks;
            VectorDynamic coeffTry = coeff;
            Reorder(tasksTry, coeffTry, "RM");
            double errCurrTry = RealObj(tasksTry, coeffTry);
            if (errCurrTry < errCurr)
            {
                tasks = tasksTry;
                coeff = coeffTry;
            }
        }

        // adjust optimization settings
        loopCount++;

        // int eliminateIteCount = 0;
        // std::vector<bool> maskForEliminationCopy = maskForElimination;
        // while (EqualVector(maskForEliminationCopy, maskForElimination) && eliminateIteCount < adjustEliminateMaxIte)
        // {
        //     FactorGraphType::FindEliminatedVariables(tasks, maskForEliminationCopy, disturbIte);
        //     disturbIte *= eliminateStep;
        //     eliminateIteCount++;
        // }
        // maskForElimination = maskForEliminationCopy;
        FactorGraphType::FindEliminatedVariables(tasks, maskForElimination);

        RoundPeriod(tasks, maskForElimination, coeff);
        errCurr = RealObj(tasks, coeff);
        if (Equals(maskForElimination, maskForEliminationPrev) && relativeErrorTolerance > relativeErrorToleranceMin)
        {
            relativeErrorTolerance = relativeErrorTolerance / 10;
        }
        if (debugMode)
        {
            cout << Color::green << "Loop " + to_string_precision(loopCount, 4) + ": " + to_string(errCurr) << Color::def << endl;
            print(maskForElimination);
            cout << endl;
        }
    }
    if (ContainFalse(maskForElimination))
    {
        UpdateTaskSetPeriod(tasks, periodResPrev);
        RoundPeriod(tasks, maskForElimination, coeff);
    }
    else
    {
        ; //nothing else to do
    }

    cout << "The number of outside loops in OptimizeTaskSetIterative is " << loopCount << endl;
    return make_pair(GetParameterVD<double>(tasks, "period"), RealObj(tasks, coeff));
}