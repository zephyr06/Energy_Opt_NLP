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
#include "sources/Utils/GlobalVariables.h"
namespace rt_num_opt
{
    // using namespace gtsam;
    /**
     * @brief returns 0 if x>=0
     *
     * @param x
     * @return double
     */
    double HingeLoss(double x)
    {
        return max(0, -1 * x);
    }

    typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
    typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;

    /**
     * @brief Constraint of x <= b
     *
     */
    class InequalityFactor1D : public gtsam::NoiseModelFactor1<VectorDynamic>
    {
    public:
        NormalErrorFunction1D f;
        int dimension;
        int index;
        /**
         * @brief Construct a new Inequality Factor 1 D object,
         *  mainly used in derived class because f is not defined
         */
        InequalityFactor1D(gtsam::Key key,
                           gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<VectorDynamic>(model, key)
        {
            index = -1;
            dimension = 1;
        }

        InequalityFactor1D(gtsam::Key key, int index,
                           gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<VectorDynamic>(model, key),
                                                            index(index)
        {
            dimension = 1;
        }

        InequalityFactor1D(gtsam::Key key, NormalErrorFunction1D f,
                           gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<VectorDynamic>(model, key),
                                                            f(f)
        {
            index = -1;
            dimension = 1;
        }

        /** active when constraint *NOT* met */
        bool active(const gtsam::Values &c) const override
        {
            if (index == -1)
            {
                // note: still active at equality to avoid zigzagging?
                VectorDynamic x = (c.at<VectorDynamic>(this->key()));
                if (exactJacobian)
                    return f(x)(0, 0) >= 0;
                else
                    return f(x)(0, 0) > 0;
            }
            else
            {
                return true;
            }
        }

        gtsam::Vector evaluateError(const VectorDynamic &x,
                                    boost::optional<gtsam::Matrix &> H = boost::none) const override
        {
            VectorDynamic err = f(x);
            if (H)
            {
                if (exactJacobian)
                {
                    *H = NumericalDerivativeDynamic(f, x, deltaOptimizer, dimension);
                }
                else
                {
                    *H = GenerateVectorDynamic(dimension);
                }

                *H = *H * jacobianScale;
            }
            return err;
        }
    };

    /**
     * @brief Constraint of x <= b
     *
     */
    class SmallerThanFactor1D : public InequalityFactor1D
    {
    public:
        double b;
        SmallerThanFactor1D(gtsam::Key key, double b,
                            gtsam::SharedNoiseModel model) : InequalityFactor1D(key, model)
        {
            f = [b](const VectorDynamic &x)
            {
                VectorDynamic res = x;
                res << Barrier(b - x(0, 0));
                return res;
            };
        }

        SmallerThanFactor1D(gtsam::Key key, double b, int indexInEliminationRecord,
                            gtsam::SharedNoiseModel model) : InequalityFactor1D(key, indexInEliminationRecord, model)
        {
            f = [b](const VectorDynamic &x)
            {
                VectorDynamic res = x;
                res << Barrier(b - x(0, 0));
                return res;
            };
        }
    };

    /**
     * @brief Constraint of x >= b
     *
     */
    class LargerThanFactor1D : public InequalityFactor1D
    {
    public:
        double b;
        LargerThanFactor1D(gtsam::Key key, double b,
                           gtsam::SharedNoiseModel model) : InequalityFactor1D(key, model)
        {
            f = [b](const VectorDynamic &x)
            {
                VectorDynamic res = x;
                res << Barrier(x(0, 0) - b);
                return res;
            };
        }

        LargerThanFactor1D(gtsam::Key key, double b, int indexInEliminationRecord,
                           gtsam::SharedNoiseModel model) : InequalityFactor1D(key, indexInEliminationRecord, model)
        {
            f = [b](const VectorDynamic &x)
            {
                VectorDynamic res = x;
                res << Barrier(x(0, 0) - b);
                return res;
            };
        }
    };

    MatrixDynamic NumericalDerivativeDynamic2D1(NormalErrorFunction2D h,
                                                const VectorDynamic &x1,
                                                const VectorDynamic &x2,
                                                double deltaOptimizer,
                                                int mOfJacobian)
    {
        int n = x1.rows();
        MatrixDynamic jacobian;
        jacobian.resize(mOfJacobian, n);
        NormalErrorFunction1D f = [h, x2](const VectorDynamic &x1)
        {
            return h(x1, x2);
        };

        return NumericalDerivativeDynamic(f, x1, deltaOptimizer, mOfJacobian);
    }
    MatrixDynamic NumericalDerivativeDynamic2D2(NormalErrorFunction2D h,
                                                const VectorDynamic &x1,
                                                const VectorDynamic &x2,
                                                double deltaOptimizer,
                                                int mOfJacobian)
    {
        int n = x2.rows();
        MatrixDynamic jacobian;
        jacobian.resize(mOfJacobian, n);
        NormalErrorFunction1D f = [h, x1](const VectorDynamic &x2)
        {
            return h(x1, x2);
        };

        return NumericalDerivativeDynamic(f, x2, deltaOptimizer, mOfJacobian);
    }

    /**
     * @brief Constraint of f(x1, x2) <= 0;
     * x1 and x2 are vectors of size (1,1)
     *
     */
    class InequalityFactor2D : public gtsam::NoiseModelFactor2<VectorDynamic, VectorDynamic>
    {
    public:
        /**
     * @brief an example of the f function
     * f = [](const VectorDynamic &x1, const VectorDynamic &x2)
         {
             return (x1 + x2);
         };
     */
        NormalErrorFunction2D f;

        InequalityFactor2D(gtsam::Key key1, gtsam::Key key2, NormalErrorFunction2D f,
                           gtsam::SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2),
                                                            f(f)
        {
        }

        /** active when constraint *NOT* met */
        bool active(const gtsam::Values &c) const override
        {
            // note: still active at equality to avoid zigzagging??
            VectorDynamic x0 = (c.at<VectorDynamic>(this->keys()[0]));
            VectorDynamic x1 = (c.at<VectorDynamic>(this->keys()[1]));
            return f(x0, x1)(0, 0) >= 0;
            // return true;
        }

        gtsam::Vector evaluateError(const VectorDynamic &x1, const VectorDynamic &x2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none) const override
        {
            VectorDynamic err = f(x1, x2);
            if (H1)
            {
                *H1 = NumericalDerivativeDynamic2D1(f, x1, x2, deltaOptimizer, 1);
            }
            if (H2)
            {
                *H2 = NumericalDerivativeDynamic2D2(f, x1, x2, deltaOptimizer, 1);
            }
            // if (err(0, 0) != 0)
            // {
            //     int a = 1;
            // }
            return err;
        }
    };
} // namespace rt_num_opt