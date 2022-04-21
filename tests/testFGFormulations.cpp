#include "../sources/ControlOptimize.h"
using namespace rt_num_opt;
double Fobj(double x)
{
    return x * x * x + sin(x) + cos(3 * x * x);
}

double Jobj(double x)
{
    return 3 * x * x + cos(x) - sin(3 * x * x) * 6 * x;
}
int Nvar = 20;

class MyTestFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    MyTestFactor1D(Key key,
                   SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key) {}
    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic err = GenerateVectorDynamic1D(Fobj(x(0)));
        if (H)
        {
            *H = GenerateVectorDynamic1D(Jobj(x(0)));
        }
        return err;
    }
};

class MyTestFactorND : public NoiseModelFactor1<VectorDynamic>
{
public:
    MyTestFactorND(Key key,
                   SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key) {}
    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        // int n = x.rows();
        VectorDynamic err = GenerateVectorDynamic(Nvar);
        for (int i = 0; i < Nvar; i++)
        {
            err(i) = Fobj(x(i));
        }
        if (H)
        {
            MatrixDynamic J = GenerateMatrixDynamic(Nvar, Nvar);
            for (int i = 0; i < Nvar; i++)
            {
                J(i, i) = Jobj(x(i));
            }
            *H = J;
            // *H = GenerateVectorDynamic1D(Jobj(x(0)));
        }
        return err;
    }
};

// TEST(multiple_small, v1)
// {
//     auto start = high_resolution_clock::now();
//     auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
//     NonlinearFactorGraph graph;
//     Values initialEstimateFG;

//     for (int i = 0; i < Nvar; i++)
//     {
//         graph.emplace_shared<MyTestFactor1D>(GenerateControlKey(i, "period"), model);
//         initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(0));
//     }
//     Values result;
//     LevenbergMarquardtParams params;
//     params.setlambdaInitial(initialLambda);
//     params.setVerbosityLM("SUMMARY");
//     params.setlambdaLowerBound(lowerLambda);
//     params.setlambdaUpperBound(upperLambda);
//     params.setRelativeErrorTol(relativeErrorTolerance);
//     LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
//     result = optimizer.optimize();
//     auto stop = high_resolution_clock::now();
//     auto duration = duration_cast<microseconds>(stop - start);
//     cout << "multiple_small:" << duration.count() << endl;
// }

// TEST(single_big, v1)
// {
//     auto start = high_resolution_clock::now();
//     auto model = noiseModel::Isotropic::Sigma(Nvar, noiseModelSigma);
//     NonlinearFactorGraph graph;
//     Values initialEstimateFG;

//     graph.emplace_shared<MyTestFactorND>(GenerateControlKey(0, "period"), model);
//     initialEstimateFG.insert(GenerateControlKey(0, "period"), GenerateVectorDynamic(Nvar));

//     Values result;
//     LevenbergMarquardtParams params;
//     params.setlambdaInitial(initialLambda);
//     params.setVerbosityLM("SUMMARY");
//     params.setlambdaLowerBound(lowerLambda);
//     params.setlambdaUpperBound(upperLambda);
//     params.setRelativeErrorTol(relativeErrorTolerance);
//     LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
//     result = optimizer.optimize();
//     auto stop = high_resolution_clock::now();
//     auto duration = duration_cast<microseconds>(stop - start);
//     cout << "single_big:" << duration.count() << endl;
// }
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
