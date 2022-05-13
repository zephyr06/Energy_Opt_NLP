#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/ControlOptimization/ReadControlCases.h"
using namespace std;
using namespace std::chrono;
using namespace gtsam;
using namespace rt_num_opt;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
using namespace ControlOptimize;
TEST(ReadControlCase1, v1)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet task1;
    VectorDynamic coeff;
    std::tie(task1, coeff) = ReadControlCase(path1);
    VectorDynamic expectCoeff = GenerateVectorDynamic(10);
    expectCoeff << 645, 7143, 275, 9334, 217, 5031, 489, 3778, 285, 380;
    VectorDynamic expectC = GenerateVectorDynamic(5);
    expectC << 2, 48, 18, 47, 12;
    VectorDynamic actualC = GetParameterVD<double>(task1, "executionTime");
    AssertEigenEqualVector(expectC, actualC);
    VectorDynamic expectT = GenerateVectorDynamic(5);
    expectT << 635, 635, 635, 635, 635;
    VectorDynamic actualT = GetParameterVD<double>(task1, "period");
    AssertEigenEqualVector(expectT, actualT);
    AssertEigenEqualVector(expectCoeff, coeff);
}

TEST(ReadControlCase1, v2)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N10/Case10.txt";
    TaskSet task1;
    VectorDynamic coeff;
    std::tie(task1, coeff) = ReadControlCase(path1);
    VectorDynamic expectCoeff = GenerateVectorDynamic(20);
    expectCoeff << 421, 1864, 564, 1346, 571, 1738, 990, 994, 42, 1810, 345, 5944, 618, 9345, 664, 8028, 885, 7367, 757, 1371;
    VectorDynamic expectC = GenerateVectorDynamic(10);
    expectC << 23, 52, 77, 66, 7, 84, 4, 54, 19, 39;
    VectorDynamic actualC = GetParameterVD<double>(task1, "executionTime");
    AssertEigenEqualVector(expectC, actualC);
    VectorDynamic expectT = GenerateVectorDynamic(10);
    expectT.array() += 2125;
    VectorDynamic actualT = GetParameterVD<double>(task1, "period");
    AssertEigenEqualVector(expectT, actualT);
    AssertEigenEqualVector(expectCoeff, coeff);
}

TEST(rta1, v1)
{
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    tasks[0].period = 68.000000;
    tasks[1].period = 129.000003;
    tasks[2].period = 129.003; // test fails if 128.9993
    tasks[3].period = 129.000002;
    tasks[4].period = 129.000004;
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    VectorDynamic expectRta = GenerateVectorDynamic(5);
    expectRta << 2, 50, 68, 117, 129;
    AssertEigenEqualVector(expectRta, rta);
}

class ControlFactorT : public NoiseModelFactor1<VectorDynamic>
{
public:
    TaskSet tasks_;
    VectorDynamic coeffVec_;
    std::vector<bool> eliminationMask_;
    int N;

    ControlFactorT(Key key, TaskSet &tasks, VectorDynamic coeffVec,
                   std::vector<bool> eliminationMask,
                   SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                             tasks_(tasks), coeffVec_(coeffVec),
                                             eliminationMask_(eliminationMask)
    {
        N = tasks_.size();
    }
    // bool active(const Values &) const override { return true; }

    VectorDynamic EstimateControlAndSchedulability(const TaskSet &tasks) const
    {
        VectorDynamic resV = GenerateVectorDynamic(tasks.size() * 5);
        RTA_LL r(tasks);
        VectorDynamic rta = r.ResponseTimeOfTaskSet();
        double totalExecution = GetParameterVD<double>(tasks, "executionTime").sum();
        for (uint i = 0; i < tasks.size(); i++)
        {
            resV(5 * i) = pow(coeffVec_[2 * i] * tasks[i].period, 1);
            resV(5 * i + 1) = pow(coeffVec_[2 * i + 1] * rta(i), 1);
            resV(5 * i + 2) = max(0, tasks[i].period - totalExecution * 5) * weightEnergy;
            resV(5 * i + 3) = max(0, 0 - tasks[i].period) * weightEnergy;
            resV(5 * i + 4) = max(0, rta(i) - min(tasks[i].deadline, tasks[i].period)) * weightEnergy;
        }
        cout << "The current error is "
             << resV.norm() << endl
             << endl;
        return resV;
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

        boost::function<Matrix(const VectorDynamic &)> f2 =
            [this](const VectorDynamic &executionTimeVector)
        {
            TaskSet taskT = tasks_;
            UpdateTaskSetPeriod(taskT, executionTimeVector);
            cout << "Current period time vector is " << endl
                 << executionTimeVector << endl;
            return EstimateControlAndSchedulability(taskT);
        };

        // boost::function<Matrix(const VectorDynamic &)> f =
        //     [&taskDurOpt, &f2, this](const VectorDynamic &executionTimeVector)
        // {
        //     UpdateTaskSetExecutionTime(taskDurOpt.tasks_, executionTimeVector, lastTaskDoNotNeedOptimize);
        //     Schedul_Analysis r(taskDurOpt);
        //     VectorDynamic responseTimeVec = r.ResponseTimeOfTaskSet(responseTimeInitial);
        //     VectorDynamic err = f2(executionTimeVector);

        //     double currentEnergyConsumption = err.sum();
        //     for (int i = 0; i < N; i++)
        //     {
        //         // barrier function part
        //         err(i, 0) += Barrier(taskDurOpt.tasks_[i].deadline - responseTimeVec(i, 0));
        //         if (enableMaxComputationTimeRestrict)
        //             err(i, 0) += Barrier(taskDurOpt.tasks_[i].executionTimeOrg * MaxComputationTimeRestrict -
        //                                  taskDurOpt.tasks_[i].executionTime);
        //     }
        //     UpdateGlobalVector(responseTimeVec, currentEnergyConsumption, taskDurOpt.tasks_);
        //     return err;
        // };

        VectorDynamic err;
        err = f2(executionTimeVector);
        if (H)
        {
            // if (exactJacobian)
            //     *H = NumericalDerivativeDynamicUpper(f, executionTimeVector, deltaOptimizer, N);
            // else
            *H = NumericalDerivativeDynamic(f2, executionTimeVector, deltaOptimizer, N * 5);
            // *H = NumericalDerivativeDynamic(f2, executionTimeVector, deltaOptimizer, numberOfTasksNeedOptimize);
            // *H = jacobian;
            // for (uint i = 0; i < N; i++)
            // {
            //     if (eliminationMask_[i])
            //     {
            //         (*H)(5 * i, 0) = 0;
            //         (*H)(5 * i + 1, 0) = 0;
            //     }
            // }
            for (uint i = 0; i < H->rows(); i++)
            {
                for (uint j = 0; j < H->cols(); j++)
                {
                    if (abs((*H)(i, j)) > 1e30)
                    {
                        // clear column
                        for (uint k = 0; k < H->rows(); k++)
                        {
                            (*H)(k, j) = 0;
                        }
                    }
                    // if (j != 2)
                    //     (*H)(i, j) = 0;
                }
            }
            // (*H)(5 * 3, 3) = 0;
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

// class

// TEST(period, error1)
// {
//     // weightEnergy = 1;

//     TaskSet tasks;
//     tasks.push_back(Task{0, 635, 0, 2, 635, 0, 0});
//     tasks.push_back(Task{0, 635, 0, 48, 635, 1, 0});
//     tasks.push_back(Task{0, 635, 0, 18, 635, 2, 0});
//     tasks.push_back(Task{0, 635, 0, 47, 635, 3, 0});
//     tasks.push_back(Task{0, 635, 0, 12, 635, 4, 0});
//     int N = tasks.size();
//     auto model = noiseModel::Isotropic::Sigma(N * 5, noiseModelSigma);

//     Symbol key('a', 0);
//     VectorDynamic coeff;
//     coeff.resize(10, 1);
//     coeff << 645, 7143, 275, 9334, 217, 5031, 489, 3778, 285, 380;
//     VectorDynamic initialEstimate = GenerateVectorDynamic(5);
//     initialEstimate << 635, 635, 635, 635, 635;
//     std::vector<bool> eliminationMask(tasks.size(), false);
//     ControlFactorT factor1(key, tasks, coeff, eliminationMask, model);
//     cout << factor1.evaluateError(initialEstimate) << endl;
//     AssertEqualScalar(2519309, factor1.evaluateError(initialEstimate).sum());
//     initialEstimate << 634.9, 635, 635, 635, 635;
//     AssertEqualScalar(2519244.5, factor1.evaluateError(initialEstimate).sum());
// }
double RealObj(TaskSet &tasks, VectorDynamic coeff)
{
    // double res = 0;
    gtsam::Symbol key('a', 0);
    auto model = gtsam::noiseModel::Isotropic::Sigma(tasks.size() * 5, 1);
    std::vector<bool> eliminationMask(tasks.size(), false);
    ControlFactorT factor(key, tasks, coeff, eliminationMask, model);
    return factor.evaluateError(GetParameterVD<double>(tasks, "period")).sum();
}

pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic coeff,
                                                   std::vector<bool> &eliminationMask, VectorDynamic &initialEstimate)
{
    int N = tasks.size();
    auto model = gtsam::noiseModel::Isotropic::Sigma(N * 5, noiseModelSigma);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Symbol key('a', 0);
    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;

    graph.emplace_shared<ControlFactorT>(key, tasks, coeff, eliminationMask, model);

    gtsam::Values initialEstimateFG;
    initialEstimateFG.insert(key, initialEstimate);

    gtsam::Values result;
    if (optimizerType == 1)
    {
        gtsam::DoglegParams params;
        // if (debugMode == 1)
        //     params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        gtsam::DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        gtsam::LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        // if (debugMode > 1 && debugMode < 5)
        params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp = result.at<VectorDynamic>(key);
    ios::sync_with_stdio(false);

    cout << Color::blue;
    cout << "After optimization, the period vector is " << endl
         << optComp << endl;
    UpdateTaskSetPeriod(tasks, initialEstimate);
    cout << "Before optimization, the total error is " << RealObj(tasks, coeff) << endl;
    UpdateTaskSetPeriod(tasks, optComp);
    cout << "The objective function is " << RealObj(tasks, coeff) << endl;
    cout << Color::def;
    return std::make_pair(optComp, RealObj(tasks, coeff));
}
VectorDynamic OptimizeTaskSetIterative(TaskSet &tasks, VectorDynamic coeff,
                                       std::vector<bool> &eliminationMask,
                                       VectorDynamic &initialEstimate, double initialError)
{
    VectorDynamic periodRes;
    double err;
    std::tie(periodRes, err) = UnitOptimizationPeriod(tasks, coeff, eliminationMask, initialEstimate);
    if (err < initialError)
    {
        periodRes = OptimizeTaskSetIterative(tasks, coeff, eliminationMask, periodRes, err);
    }
    return periodRes;
}
bool EliminateTask(TaskSet &tasks, std::vector<bool> &eliminationMask, VectorDynamic &currPeriod)
{
    TaskSet tasksCurr = tasks;
    UpdateTaskSetPeriod(tasksCurr, currPeriod);
    RTA_LL r(tasks);
    for (uint i = 0; i < tasks.size(); i++)
    {
        tasksCurr[i].period -= eliminateTol;

        // double rt = Schedul_Analysis::RTA_Common_Warm(computationTimeWarmStart(i, 0), tasksCurr, i);
        double tolerance = 0.0;
        RTA_LL r(tasksCurr);
        bool schedulable = r.CheckSchedulability(debugMode == 1);
        if ((!schedulable) ||
            (enableMaxComputationTimeRestrict &&
             tasksCurr[i].period - eliminateTol > tasks[i].periodOrg))
        {
            eliminationMask[i] = true;
            return true;
        }
    }
    return false;
}
// TEST(optimizeperiod1, v1)
// {
//     // weightEnergy = 1e4;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     VectorDynamic initialEstimate = GenerateVectorDynamic(tasks.size()).array() + tasks[0].period;
//     initialEstimate << 68, 129.002, 218.352, 129, 128.895;
//     std::vector<bool> eliminationMask(tasks.size(), false);
//     VectorDynamic resOne;
//     double err;
//     std::tie(resOne, err) = UnitOptimizationPeriod(tasks, coeff, eliminationMask, initialEstimate);
// }

// TEST(OptimizePeriod, v2)
// {
//     // weightEnergy = 1e4;
//     std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     VectorDynamic initialEstimate = GenerateVectorDynamic(tasks.size()).array() + tasks[0].period;
//     // initialEstimate << 68, 321, 400, 129, 308;
//     initialEstimate << 68, 129.002, 218.352, 129, 128.895;
//     std::vector<bool> eliminationMask(tasks.size(), false);
//     VectorDynamic resOneTime;
//     double err;
//     std::tie(resOneTime, err) = UnitOptimizationPeriod(tasks, coeff, eliminationMask, initialEstimate);
//     bool useless = EliminateTask(tasks, eliminationMask, resOneTime);
//     AssertBool(true, useless);
//     AssertBool(true, eliminationMask[0]);
// }
TEST(OptimizeIterative, v1)
{
    std::ios::sync_with_stdio(false);
    // weightEnergy = 1e4;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    VectorDynamic initialEstimate = GenerateVectorDynamic(tasks.size()).array() + tasks[0].period;
    // initialEstimate << 68, 129, 129, 129, 128.895;
    cout.precision(17);
    // initialEstimate << 68.0001, 129.001, 218.688, 129, 128.999998;
    // initialEstimate << 68.0001, //68.000081893411362
    //     129.00105,
    //     218.68764792734831,
    //     129.00009184509682,
    //     128.999998; //128.99998098915873
    std::vector<bool> eliminationMask(tasks.size(), false);
    // VectorDynamic resOne = OptimizeTaskSetIterative(tasks, coeff, eliminationMask, initialEstimate, 1e20);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
