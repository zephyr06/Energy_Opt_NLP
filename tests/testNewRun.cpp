#include <chrono>

#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
using namespace std::chrono;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
class ControlFactor : public NoiseModelFactor1<VectorDynamic>
{
public:
    TaskSetNormal tasks_;
    int lastTaskDoNotNeedOptimize;
    VectorDynamic responseTimeInitial;
    int N;

    ControlFactor(Key key, TaskSetNormal &tasks, int lastTaskDoNotNeedOptimize, VectorDynamic responseTimeInitial,
                  SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                            tasks_(tasks), lastTaskDoNotNeedOptimize(lastTaskDoNotNeedOptimize),
                                            responseTimeInitial(responseTimeInitial)
    {
        N = tasks_.tasks_.size();
    }

    /**
         * @brief 
         * 
         * @param periodVector (numberOfTasksNeedOptimize, 1)
         * @param H 
         * @return Vector 
         */
    Vector evaluateError(const VectorDynamic &periodVector, boost::optional<Matrix &> H = boost::none) const override
    {

        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &periodVector)
        {
            TaskSetNormal tasksC = tasks_;
            UpdateTaskSetPeriod(tasksC.tasks_, periodVector);
            RTA_LL r(tasksC);
            VectorDynamic rta = r.ResponseTimeOfTaskSet();
            VectorDynamic err = rta + periodVector * frequencyRatio;
            for (int i = 0; i < N; i++)
                err(i) += Barrier(periodVector(i) - rta(i));
            return err;
        };
        if (H)
        {
            cout << Color::blue << endl;
            *H = NumericalDerivativeDynamic(f, periodVector, deltaOptimizer, N);
            cout << "The current variables are: " << endl
                 << periodVector << endl;
            cout << "The current Jacobian are: " << endl
                 << *H << endl;
            TaskSetNormal tasksC = tasks_;
            UpdateTaskSetPeriod(tasksC.tasks_, periodVector);
            RTA_LL r(tasksC);
            VectorDynamic rta = r.ResponseTimeOfTaskSet();
            cout << "rta is" << endl
                 << rta << endl;
            cout << Color::def << endl;
        }
        return f(periodVector);
    }
};

VectorDynamic UnitOpt(TaskSet &tasks, VectorDynamic &initialEstimate)
{
    TaskSetNormal tasksN(tasks);
    int N = tasks.size();
    int lastTaskDoNotNeedOptimize = -1;
    RTA_LL r(tasks);
    VectorDynamic responseTimeInitial = r.ResponseTimeOfTaskSet();
    // build the factor graph
    auto model = noiseModel::Isotropic::Sigma(N, noiseModelSigma);
    NonlinearFactorGraph graph;
    Symbol key('a', 0);
    graph.emplace_shared<ControlFactor>(key, tasksN, lastTaskDoNotNeedOptimize, responseTimeInitial, model);

    // VectorDynamic initialEstimate = GetParameterVD<double>(tasks, "period");
    Values initialEstimateFG;
    initialEstimateFG.insert(key, initialEstimate);

    Values result;
    if (optimizerType == 1)
    {
        DoglegParams params;
        if (debugMode == 1)
            params.setVerbosityDL("SUMMARY");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        if (debugMode >= 1 && debugMode < 5)
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
    cout << "After optimization for one iteration, the result is " << optComp << endl;
    UpdateTaskSetPeriod(tasks, optComp);
    RTA_LL r2(tasks);
    VectorDynamic rta = r2.ResponseTimeOfTaskSet();
    cout << "After optimization for one iteration, the response time is " << endl
         << rta << endl;
    return optComp;
}
TEST(control, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v23.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    VectorDynamic initial0 = GetParameterVD<double>(tasks, "period");
    auto x1 = UnitOpt(tasks, initial0);
    auto x2 = UnitOpt(tasks, x1);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
