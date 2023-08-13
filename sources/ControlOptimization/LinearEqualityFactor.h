#pragma once
#include <boost/optional.hpp>

#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/Parameters.h"

namespace rt_num_opt {

class LinearEqualityFactor
    : public gtsam::NoiseModelFactor2<VectorDynamic, VectorDynamic> {
    NormalErrorFunction2D f;

   public:
    LinearEqualityFactor(gtsam::Key key1, gtsam::Key key2,
                         gtsam::SharedNoiseModel model)
        : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2) {
        f = [](const VectorDynamic &x1, const VectorDynamic &x2) {
            return GenerateVectorDynamic1D((x1 - x2).norm());
        };
    }

    gtsam::Vector evaluateError(
        const VectorDynamic &x1, const VectorDynamic &x2,
        boost::optional<gtsam::Matrix &> H1 = boost::none,
        boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
        VectorDynamic err = f(x1, x2);
        if (H1) {
            *H1 = NumericalDerivativeDynamic2D1(f, x1, x2, deltaOptimizer, 1);
        }
        if (H2) {
            *H2 = NumericalDerivativeDynamic2D2(f, x1, x2, deltaOptimizer, 1);
        }
        return err;
    }
};

}  // namespace rt_num_opt