#include "../sources/ControlOptimize.h"
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;

pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic coeff,
                                                   std::vector<bool> &maskForElimination)
{
    NonlinearFactorGraph graph = BuildControlGraph(maskForElimination, tasks, coeff);

    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    Values initialEstimateFG = GenerateInitialFG(tasks, maskForElimination);

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
        // if (debugMode > 1 && debugMode < 5)
        params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp, rtaFromOpt;
    std::tie(optComp, rtaFromOpt) = ExtractResults(result);
    cout << endl;
    cout << Color::blue;
    cout << "After optimization, the period vector is " << endl
         << optComp << endl;
    cout << "After optimization, the rta vector is " << endl
         << rtaFromOpt << endl;
    cout << Color::def;
    cout << endl;
    cout << Color::blue;
    UpdateTaskSetPeriod(tasks, ExtractResults(initialEstimateFG).first);
    cout << "Before optimization, the total error is " << realObj(tasks, coeff) << endl;
    UpdateTaskSetPeriod(tasks, optComp);
    cout << "The objective function is " << realObj(tasks, coeff) << endl;
    cout << Color::def;

    double eeee = graph.error(result);

    return make_pair(optComp, realObj(tasks, coeff));
}
VectorDynamic OptimizeTaskSetIterative(TaskSet &tasks, VectorDynamic coeff,
                                       std::vector<bool> &maskForElimination,
                                       double initialError)
{
    VectorDynamic periodRes;
    double err;
    std::tie(periodRes, err) = UnitOptimizationPeriod(tasks, coeff, maskForElimination);
    if (err < initialError)
    {
        periodRes = OptimizeTaskSetIterative(tasks, coeff, maskForElimination, err);
    }
    return periodRes;
}

void FindEliminatedVariables(TaskSet &tasks, std::vector<bool> &maskForElimination)
{
    RTA_LL r(tasks);
    VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        tasks[i].period -= deltaOptimizer;
        RTA_LL r1(tasks);
        VectorDynamic rtaCurr = r.ResponseTimeOfTaskSet();
        if ((rtaBase - rtaCurr).array().abs().maxCoeff() >= eliminateTol)
        {
            maskForElimination[i] = true;
        }
        tasks[i].period += deltaOptimizer;
    }
}

TEST(case1, v1)
{
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    auto sth = UnitOptimizationPeriod(tasks, coeff, maskForElimination);
    UpdateTaskSetPeriod(tasks, sth.first);
    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false}, maskForElimination);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
