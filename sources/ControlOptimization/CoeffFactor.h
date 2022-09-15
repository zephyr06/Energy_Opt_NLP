#pragma once

#include <boost/function.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "sources/Tools/testMy.h"
#include "sources/Tools/profilier.h"
#include "sources/Utils/Parameters.h"
namespace rt_num_opt
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
    typedef long long int LLint;

    /**
     * @brief error= coeff_ * x
     *
     */
    class CoeffFactor : public gtsam::NoiseModelFactor1<VectorDynamic>
    {
    public:
        VectorDynamic coeff_;

        CoeffFactor(gtsam::Key key, VectorDynamic coeff,
                    gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor1<VectorDynamic>(model, key),
                                                     coeff_(coeff)
        {
            if (coeff_.cols() == 1 && coeff_.rows() >= 1)
            {
                coeff_ = coeff_.transpose();
            }
        }

        gtsam::Vector evaluateError(const VectorDynamic &x,
                                    boost::optional<gtsam::Matrix &> H = boost::none) const override
        {
            BeginTimer("CoeffFactor");
            AssertEqualScalar(coeff_.rows(), x.rows(), 1e-6, __LINE__);

            if (H)
            {
                *H = coeff_;
                if (!whether_ls)
                    (*H)(0) = 0.5 * pow(coeff_(0) / x(0), 0.5);

                if (coeff_.rows() != 1)
                {
                    CoutError("Wrong dimension of Jacobian in CoeffFactor!");
                }
                *H = *H * jacobianScale;
            }
            VectorDynamic err = coeff_ * (x);

            if (!whether_ls)
            {
                err(0) = pow(err(0), 0.5);
            }

            EndTimer("CoeffFactor");
            return err;
        }
    };
} // namespace rt_num_opt