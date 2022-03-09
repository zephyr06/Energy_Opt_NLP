#pragma once

#include <boost/function.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;

/**
 * @brief error= coeff_ * x
 * 
 */
class CoeffFactor : public NoiseModelFactor1<VectorDynamic>
{
public:
    VectorDynamic coeff_;

    CoeffFactor(Key key, VectorDynamic coeff,
                SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                          coeff_(coeff)
    {
        if (coeff_.cols() == 1 && coeff_.rows() >= 1)
        {
            coeff_ = coeff_.transpose();
        }
    }
    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        AssertEqualScalar(coeff_.rows(), x.rows(), 1e-6, __LINE__);
        VectorDynamic err = coeff_ * (x);
        if (H)
        {
            *H = coeff_;
            if (coeff_.rows() != 1)
            {
                CoutError("Wrong dimension of Jacobian in CoeffFactor!");
            }
        }
        return err;
    }
};