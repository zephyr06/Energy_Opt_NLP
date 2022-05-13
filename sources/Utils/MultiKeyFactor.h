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

#include "sources/Utils/Parameters.h"
#include "sources/Tools/colormod.h"
#include "sources/Tools/testMy.h"
#include "sources/Utils/utils.h"
// #include "sources/Utils/FactorGraphUtils.h"
namespace rt_num_opt
{

    // using namespace gtsam;

    // typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
    // typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;
    typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
    // typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;
    typedef boost::function<gtsam::Vector(const gtsam::Values &x)> LambdaMultiKey;

    typedef long long int LLint;

    typedef std::vector<VectorDynamic> VVec;

    class MultiKeyFactor : public gtsam::NoiseModelFactor
    {
    public:
        std::vector<gtsam::Symbol> keyVec;
        uint dimension;
        LambdaMultiKey lambdaMK;

        MultiKeyFactor(std::vector<gtsam::Symbol> keyVec, LambdaMultiKey lambdaMK,
                       gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor(model, keyVec),
                                                        keyVec(keyVec),
                                                        dimension(keyVec.size()), lambdaMK(lambdaMK)
        {
        }
        /* no need to optimize if it contains no keys */
        bool active(const gtsam::Values &c) const override
        {
            return keyVec.size() != 0;
        }
        gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                      boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
        {
            BeginTimer("MultiKeyFactor");
            if (H)
            {
                for (uint i = 0; i < dimension; i++)
                {
                    NormalErrorFunction1D f =
                        [x, i, this](const VectorDynamic xi)
                    {
                        gtsam::Values xx = x;
                        xx.update(keyVec.at(i), xi);
                        return lambdaMK(xx);
                    };
                    (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                }
                if (debugMode == 1)
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    std::cout << Color::blue;
                    // PrintControlValues(x);
                    // x.print();
                    std::cout << Color::def;
                }
            }
            EndTimer("MultiKeyFactor");
            return lambdaMK(x);
        }
    };

} // namespace rt_num_opt