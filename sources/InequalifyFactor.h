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
#include "GlobalVariables.h"

using namespace std;
using namespace gtsam;

typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;

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

/**
 * @brief Constraint of x <= b
 *
 */
class InequalityFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    NormalErrorFunction1D f;
    int dimension;
    int indexInEliminationRecord;
    /**
     * @brief Construct a new Inequality Factor 1 D object,
     *  mainly used in derived class because f is not defined
     */
    InequalityFactor1D(Key key,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key)
    {
        dimension = 1;
    }

    InequalityFactor1D(Key key, int indexInEliminationRecord,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 indexInEliminationRecord(indexInEliminationRecord)
    {
        dimension = 1;
    }

    InequalityFactor1D(Key key, NormalErrorFunction1D f,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 f(f)
    {
        dimension = 1;
    }

    /** active when constraint *NOT* met */
    bool active(const Values &c) const override
    {
        // note: still active at equality to avoid zigzagging?
        VectorDynamic x = (c.at<VectorDynamic>(this->key()));
        return f(x)(0, 0) > 0;
    }

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic err = f(x);

        eliminationRecordGlobal.AdjustEliminationError(err(0), indexInEliminationRecord, EliminationType::Bound);

        if (H)
        {
            *H = NumericalDerivativeDynamic(f, x, deltaOptimizer, dimension);
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
    SmallerThanFactor1D(Key key, double b,
                        SharedNoiseModel model) : InequalityFactor1D(key, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << HingeLoss(b - x(0, 0));
            return res;
        };
    }

    SmallerThanFactor1D(Key key, double b, int indexInEliminationRecord,
                        SharedNoiseModel model) : InequalityFactor1D(key, indexInEliminationRecord, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << HingeLoss(b - x(0, 0));
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
    LargerThanFactor1D(Key key, double b,
                       SharedNoiseModel model) : InequalityFactor1D(key, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << HingeLoss(x(0, 0) - b);
            return res;
        };
    }

    LargerThanFactor1D(Key key, double b, int indexInEliminationRecord,
                       SharedNoiseModel model) : InequalityFactor1D(key, indexInEliminationRecord, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << HingeLoss(x(0, 0) - b);
            // if (res(0, 0) != 0)
            // {
            //     int a = 1;
            // }
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
class InequalityFactor2D : public NoiseModelFactor2<VectorDynamic, VectorDynamic>
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

    InequalityFactor2D(Key key1, Key key2, NormalErrorFunction2D f,
                       SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2),
                                                 f(f)
    {
    }

    /** active when constraint *NOT* met */
    bool active(const Values &c) const override
    {
        // note: still active at equality to avoid zigzagging??
        VectorDynamic x0 = (c.at<VectorDynamic>(this->keys()[0]));
        VectorDynamic x1 = (c.at<VectorDynamic>(this->keys()[1]));
        return f(x0, x1)(0, 0) >= 0;
        // return true;
    }

    Vector evaluateError(const VectorDynamic &x1, const VectorDynamic &x2,
                         boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const override
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