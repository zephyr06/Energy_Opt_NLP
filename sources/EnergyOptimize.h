
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Declaration.h"
#include "../sources/ReadControlCases.h"
#include "../sources/RTAFactor.h"
#include "../sources/InequalifyFactor.h"

#include "ControlFactorGraphUtils.h"
#include "FactorGraphEnergyLL.h"
#include <chrono>
using namespace std::chrono;

namespace EnergyOptimize
{
    template <typename FactorGraphType>
    pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks,
                                                       std::vector<bool> &maskForElimination)
    {
        BeginTimer(__func__);
        NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(maskForElimination, tasks);
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

        auto start = high_resolution_clock::now();
        auto sth = graph.error(initialEstimateFG);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "Evaluate error:" << duration.count() << endl;
        start = high_resolution_clock::now();
        auto sth2 = graph.linearize(initialEstimateFG);
        stop = high_resolution_clock::now();
        duration = duration_cast<microseconds>(stop - start);
        cout << "linearize:" << duration.count() << endl;

        cout << Color::green;
        // std::lock_guard<std::mutex> lock(mtx);
        auto sth3 = graph.linearize(initialEstimateFG)->jacobian();
        MatrixDynamic jacobianCurr = sth3.first;
        std::cout << "Current Jacobian matrix:" << endl;
        std::cout << jacobianCurr << endl;
        std::cout << "Current b vector: " << endl;
        std::cout << sth3.second << endl;
        cout << Color::def << endl;

        VectorDynamic optComp, rtaFromOpt; // rtaFromOpt can only be used for 'cout'
        optComp = FactorGraphType::ExtractResults(result, tasks);
        UpdateTaskSetExecutionTime(tasks, optComp);
        rtaFromOpt = RTALLVector(tasks);
        if (debugMode == 1)
        {
            cout << endl;
            cout << Color::blue;
            cout << "After optimization, the executionTime vector is " << endl
                 << optComp << endl;
            cout << "After optimization, the rta vector is " << endl
                 << rtaFromOpt << endl;
            cout << "The graph error is " << graph.error(result) << endl;
            cout << Color::def;
            cout << endl;
            cout << Color::blue;
            VectorDynamic newExecutionTime = FactorGraphEnergyLL::ExtractResults(initialEstimateFG, tasks);
            UpdateTaskSetExecutionTime(tasks, newExecutionTime);
            cout << "Before optimization, the total error is " << FactorGraphType::RealObj(tasks) << endl;
            UpdateTaskSetExecutionTime(tasks, optComp);
            cout << "After optimization, the total error is " << FactorGraphType::RealObj(tasks) << endl;
            cout << Color::def;
        }

        // UpdateTaskSetExecutionTime(tasks, optComp);
        EndTimer(__func__);
        return make_pair(optComp, FactorGraphType::RealObj(tasks));
    }

    template <typename FactorGraphType>
    pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(TaskSet &tasks,
                                                               std::vector<bool> &maskForElimination)
    {
        RTA_LL rr(tasks);
        if (!rr.CheckSchedulability(debugMode == 1))
        {
            cout << "The task set is not schedulable!" << endl;
            return make_pair(GetParameterVD<double>(tasks, "executionTime"), 1e30);
        }
        VectorDynamic periodRes;
        double err;
        for (double weight = weightSchedulabilityMin; weight <= weightSchedulabilityMax;
             weight *= weightSchedulabilityStep)
        {
            weightSchedulability = weight;
            std::tie(periodRes, err) = UnitOptimizationPeriod<FactorGraphType>(tasks, maskForElimination);
            VectorDynamic periodPrev = GetParameterVD<double>(tasks, "executionTime");
            UpdateTaskSetExecutionTime(tasks, periodRes);
            RTA_LL r(tasks);
            if (!r.CheckSchedulability(1 == debugMode))
            {
                UpdateTaskSetExecutionTime(tasks, periodPrev);
                if (debugMode == 1)
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cout << Color::blue << "After one iterate on updating weight parameter,\
             the execution time become unschedulable and are"
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
             the execution time remain schedulable and are"
                         << endl
                         << periodRes << endl;
                    cout << Color::def;
                }
            }
        }

        return make_pair(periodRes, err);
    }

    /**
 * @brief round period into int type, we assume the given task set is schedulable!
 *
 * @param tasks
 * @param maskForElimination
 * @param coeff
 */
    void RoundExecutionTime(TaskSet &tasks, std::vector<bool> &maskForElimination)
    {
        if (roundTypeInClamp == "none")
            return;
        else if (roundTypeInClamp == "rough")
        {
            for (uint i = 0; i < tasks.size(); i++)
            {

                tasks[i].executionTime = int(tasks[i].executionTime);
            }
        }
        // else if (roundTypeInClamp == "fine")
        // {
        //     int N = tasks.size();

        //     vector<int> wait_for_eliminate_index;
        //     for (uint i = 0; i < tasks.size(); i++)
        //     {
        //         if (maskForElimination[i] && tasks[i].period != ceil(tasks[i].period))
        //             wait_for_eliminate_index.push_back(i);
        //     }
        //     if (!wait_for_eliminate_index.empty())
        //     {

        //         VectorDynamic rtaBase = RTALLVector(tasks);

        //         vector<pair<int, double>> objectiveVec;
        //         objectiveVec.reserve(wait_for_eliminate_index.size());
        //         for (uint i = 0; i < wait_for_eliminate_index.size(); i++)
        //         {
        //             objectiveVec.push_back(make_pair(wait_for_eliminate_index[i], coeff(wait_for_eliminate_index[i] * 2) * -1));
        //         }
        //         sort(objectiveVec.begin(), objectiveVec.end(), comparePair);
        //         int iterationNumber = 0;

        //         // int left = 0, right = 0;
        //         while (objectiveVec.size() > 0)
        //         {
        //             int currentIndex = objectiveVec[0].first;

        //             // try to round 'up', if success, keep the loop; otherwise, eliminate it and high priority tasks
        //             // can be speeded up, if necessary, by binary search
        //             int left = int(tasks[currentIndex].period);
        //             // int left = rtaBase(currentIndex);
        //             int right = ceil(tasks[currentIndex].period);
        //             if (left > right)
        //             {
        //                 CoutError("left > right error in clamp!");
        //             }
        //             int rightOrg = right;
        //             bool schedulale_flag;
        //             while (left < right)
        //             {
        //                 int mid = (left + right) / 2;

        //                 tasks[currentIndex].period = mid;
        //                 RTA_LL r(tasks);
        //                 schedulale_flag = r.CheckSchedulability(
        //                     rtaBase, debugMode == 1);
        //                 if (not schedulale_flag)
        //                 {
        //                     left = mid + 1;
        //                     tasks[currentIndex].period = rightOrg;
        //                 }
        //                 else
        //                 {
        //                     tasks[currentIndex].period = mid;
        //                     right = mid;
        //                 }
        //             }

        //             // post processing, left=right is the value we want
        //             tasks[currentIndex].period = left;
        //             objectiveVec.erase(objectiveVec.begin() + 0);

        //             iterationNumber++;
        //             if (iterationNumber > N)
        //             {
        //                 CoutWarning("iterationNumber error in Clamp!");
        //                 break;
        //             }
        //         };
        //     }
        // }
        else
        {
            cout << "input error in ClampComputationTime: " << roundTypeInClamp << endl;
            throw;
        }
        return;
    }

    // TODO: limit the number of outer loops
    template <typename FactorGraphType>
    pair<VectorDynamic, double> OptimizeTaskSetIterative(TaskSet &tasks)
    {

        VectorDynamic periodResCurr, periodResPrev;
        std::vector<bool> maskForElimination(tasks.size(), false);
        std::vector<bool> maskForEliminationPrev = maskForElimination;
        double errPrev = 1e30;
        double errCurr = FactorGraphType::RealObj(tasks);
        int loopCount = 0;
        double disturbIte = eliminateTol;
        while (ContainFalse(maskForElimination) && loopCount < MaxLoopControl) //errCurr < errPrev * (1 - relativeErrorToleranceOuterLoop) &&
        {
            // store prev result
            errPrev = errCurr;
            periodResPrev = GetParameterVD<double>(tasks, "executionTime");
            maskForEliminationPrev = maskForElimination;

            // perform optimization
            double err;
            std::tie(periodResCurr, err) = OptimizeTaskSetIterativeWeight<FactorGraphType>(tasks, maskForElimination);
            UpdateTaskSetExecutionTime(tasks, periodResCurr);

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
            disturbIte = FactorGraphType::FindEliminatedVariables(tasks, maskForElimination, disturbIte);

            RoundExecutionTime(tasks, maskForElimination);
            errCurr = FactorGraphType::RealObj(tasks);
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
            UpdateTaskSetExecutionTime(tasks, periodResPrev);
            RoundExecutionTime(tasks, maskForElimination);
        }
        else
        {
            ; //nothing else to do
        }

        cout << "The number of outside loops in OptimizeTaskSetIterative is " << loopCount << endl;
        return make_pair(GetParameterVD<double>(tasks, "executionTime"), FactorGraphType::RealObj(tasks));
    }
}