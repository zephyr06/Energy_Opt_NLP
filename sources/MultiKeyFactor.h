#pragma once

#include <chrono>
#include <unordered_map>
#include <math.h>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <dirent.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/function.hpp>

#include "Parameters.h"
#include "colormod.h"
#include "testMy.h"
#include "utils.h"

using namespace std;
using namespace gtsam;

// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
// typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;
typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
// typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;
typedef boost::function<Vector(const Values &x)> LambdaMultiKey;

typedef long long int LLint;

typedef std::vector<VectorDynamic> VVec;

class MultiKeyFactor : public NoiseModelFactor
{
public:
    vector<Symbol> keyVec;
    uint dimension;
    LambdaMultiKey lambdaMK;

    MultiKeyFactor(vector<Symbol> keyVec, LambdaMultiKey lambdaMK,
                   SharedNoiseModel model) : NoiseModelFactor(model, keyVec),
                                             keyVec(keyVec),
                                             dimension(keyVec.size()), lambdaMK(lambdaMK)
    {
    }

    Vector unwhitenedError(const Values &x,
                           boost::optional<std::vector<Matrix> &> H = boost::none) const override
    {
        // const VectorDynamic &x0 = x.at<VectorDynamic>(keyVec[0]);
        // const VectorDynamic &x1 = x.at<VectorDynamic>(keyVec[1]);

        // VectorDynamic res = GenerateVectorDynamic(dimension);
        // for (uint i = 0; i < dimension; i++)
        // {
        //     res(i, 0) = bv[i] - x.at<VectorDynamic>(keyVec[i])(0, 0);
        // }
        // cout << "Res: " << res << endl;
        if (H)
        {
            for (uint i = 0; i < dimension; i++)
            {
                NormalErrorFunction1D f =
                    [x, i, this](const VectorDynamic xi)
                {
                    Symbol a = keyVec.at(i);
                    Values xx = x;
                    xx.update(a, xi);
                    return lambdaMK(xx);
                };
                (*H)[i] = NumericalDerivativeDynamicUpper(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer, 2);
            }
        }
        return lambdaMK(x);
    }
};