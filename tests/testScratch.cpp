#include "../sources/Optimize.h"
// #include "/home/zephyr/Library/eigen/Eigen/Core"
int TASK_NUMBER_DYNAMIC = 2;
class ComputationFactorTEST : public NoiseModelFactor1<VectorDynamic>
{
public:
    TaskSet tasks_;
    ComputationTimeVector responseTimeBase_;
    int variablesNumber;

    ComputationFactorTEST(Key key, TaskSet &tasks, int n,
                          SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                    tasks_(tasks), variablesNumber(n)
    {
        for (int i = 0; i < TASK_NUMBER; i++)
        {
            vector<Task> hpTasks;
            for (int j = 0; j < i; j++)
                hpTasks.push_back(tasks_[j]);

            responseTimeBase_[i] = ResponseTimeAnalysis<double>(tasks_[i], hpTasks);
        }
    }

    Vector evaluateError(const VectorDynamic &executionTimeVector, boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic err;

        err.resize(variablesNumber * 2, 1);
        for (int i = 0; i < variablesNumber; i++)
        {
            err(i, 0) = pow(executionTimeVector(i), 2);
            err(i + variablesNumber, 0) = pow(executionTimeVector(i), 2);
        }

        // JacobianOpt jacobian;
        // jacobian.setZero();
        // TaskSet taskSetCurr_ = tasks_;
        // UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector);

        // long long int hyperPeriod = 0;
        // // if (H)
        // //     hyperPeriod = HyperPeriod(tasks_);

        // err = EstimateEnergyTaskSetRest(tasks_, executionTimeVector);
        // todo: find a way to test the evaluteError function
        // for (int i = 0; i < TASK_NUMBER; i++)
        // {
        //     Task taskCurr_(tasks_[i]);

        //     taskCurr_.executionTime = executionTimeVector(i, 0);
        //     vector<Task> hpTasks;
        //     for (int j = 0; j < i; j++)
        //         hpTasks.push_back(taskSetCurr_[j]);

        //     double responseTime = ResponseTimeAnalysisWarm<double>(responseTimeBase_[i], taskCurr_, hpTasks);

        //     err(i, 0) += Barrier(tasks_[i].deadline - responseTime);
        // approximate the Jacobian
        // if (H)
        // {
        //     if (tasks_[i].deadline - responseTime > 0)
        //     {
        //         jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
        //         jacobian(i, i) -= 1 / (tasks_[i].deadline - responseTime);
        //         for (int j = 0; j < i; j++)
        //         {
        //             jacobian(i, j) = -1 / (tasks_[i].deadline - responseTime) * ceil(responseTime / tasks_[j].period);
        //         }
        //     }
        //     else if (tasks_[i].deadline - responseT3844
        // 3844ime <= 0)
        //     {
        //         jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
        //         jacobian(i, i) += punishmentInBarrier * pow(10, TASK_NUMBER - 3) * 1;
        //         for (int j = 0; j < i; j++)
        //         {
        //             jacobian(i, j) = punishmentInBarrier * pow(10, TASK_NUMBER - 3) * ceil(responseTime / tasks_[j].period);
        //         }
        //     }
        //     else // tasks_[i].deadline - responseTime == 0
        //     {
        //         jacobian(i, i) = -2 * hyperPeriod / tasks_[i].period * pow(tasks_[i].executionTime / executionTimeVector(i, 0), 3) * weightEnergy;
        //         jacobian(i, i) -= 1 / (tasks_[i].deadline - responseTime + toleranceBarrier);
        //         for (int j = 0; j < i; j++)
        //         {

        //             jacobian(i, j) = -1 / (tasks_[i].deadline - responseTime + toleranceBarrier) * ceil(responseTime / tasks_[j].period);
        //         }
        //     }
        // }
        // }

        if (H)
        {
            boost::function<Matrix(const VectorDynamic &)> f =
                [this](const VectorDynamic &input)
            {
                VectorDynamic err;
                err.resize(variablesNumber * 2, 1);
                for (int i = 0; i < variablesNumber; i++)
                {
                    err(i, 0) = pow(input(i), 2);
                    err(i + variablesNumber, 0) = pow(input(i), 2);
                }
                return err;
            };

            *H = NumericalDerivativeDynamic(f, executionTimeVector, deltaOptimizer, TASK_NUMBER_DYNAMIC * 2);
            // *H = jacobian;

            cout << "The current evaluation point is " << endl
                 << executionTimeVector << endl;
            cout << "The Jacobian is " << endl
                 << *H << endl;
            // cout << "The approximated Jacobian is " << endl
            //      << jacobian << endl;
            cout << "The current error is " << endl
                 << err << endl
                 << err.norm() << endl
                 << endl;
        }
        return err;
    }
};

VectorDynamic InitializeOptimizationTEST(const TaskSet &tasks)
{
    VectorDynamic comp;
    comp.resize(TASK_NUMBER_DYNAMIC, 1);
    for (int i = 0; i < TASK_NUMBER_DYNAMIC; i++)
        comp(i, 0) = tasks[tasks.size() - TASK_NUMBER_DYNAMIC + i].executionTime;
    return comp;
}
/**
 * Perform optimization for one task set
 **/
double OptimizeTaskSetTEST(TaskSet &tasks)
{
    // test schedulability
    if (!CheckSchedulability<int>(tasks))
    {
        cout << "The task set is not schedulable!\n";
        return -1;
    }

    // build the factor graph since there
    TASK_NUMBER_DYNAMIC = 2;

    auto model = noiseModel::Isotropic::Sigma(TASK_NUMBER_DYNAMIC * 2, noiseModelSigma);

    NonlinearFactorGraph graph;
    Symbol key('a', 0);

    graph.emplace_shared<ComputationFactorTEST>(key, tasks, TASK_NUMBER_DYNAMIC, model);

    // ComputationTimeVector comp;
    // comp << 10, 20, 30;
    Values initialEstimate;
    initialEstimate.insert(key, InitializeOptimizationTEST(tasks));

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

    VectorDynamic optComp = result.at<VectorDynamic>(key);
    cout << "After optimization, the computation time vector is " << optComp << endl;
    // ClampComputationTime(optComp);

    // TaskSet tasks2 = tasks;
    // UpdateTaskSetExecutionTime(tasks2, optComp);
    // bool a = CheckSchedulability<int>(tasks2);
    // if (a)
    // {
    //     cout << "The task set is schedulable after optimization\n";
    //     double initialEnergyCost = EstimateEnergyTaskSet(tasks, InitializeOptimization(tasks)).sum();
    //     double afterEnergyCost = EstimateEnergyTaskSet(tasks, optComp).sum();
    //     return afterEnergyCost / initialEnergyCost;
    // }
    // else
    // {
    //     cout << "Unfeasible!" << endl;
    //     return -1;
    // }
    return 0;
}

TEST(ComputationFactor, a1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v4.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_data_N5_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v2.csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n20_v1.csv";

    TaskSet taskSet1 = ReadTaskSet(path, "RM");

    auto res = OptimizeTaskSetTEST(taskSet1);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
