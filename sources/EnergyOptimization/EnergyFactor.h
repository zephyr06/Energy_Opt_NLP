#pragma once
#include <boost/optional.hpp>

#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "sources/EnergyOptimization/Energy.h"
#include "sources/EnergyOptimization/FrequencyModel.h"
#include "sources/Utils/Parameters.h"

namespace rt_num_opt {
class EnergyFactor : public gtsam::NoiseModelFactor1<VectorDynamic> {
   public:
    int taskIndex;
    Task task_;

    /**
     * @brief Construct a new Inequality Factor 1 D object,
     *  mainly used in derived class because f is not defined
     */
    EnergyFactor(gtsam::Key key, Task task, int index,
                 gtsam::SharedNoiseModel model)
        : gtsam::NoiseModelFactor1<VectorDynamic>(model, key),
          taskIndex(index),
          task_(task) {}

    gtsam::Vector evaluateError(
        const VectorDynamic &x,
        boost::optional<gtsam::Matrix &> H = boost::none) const override {
        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector) {
                Task taskCurr = task_;
                taskCurr.executionTime = executionTimeVector(0, 0);
                VectorDynamic err;
                if (!whether_ls) {
                    err = GenerateVectorDynamic1D(pow(
                        1.0 / taskCurr.period * EstimateEnergyTask(taskCurr),
                        0.5));
                } else {
                    err = GenerateVectorDynamic1D(pow(
                        1.0 / taskCurr.period * EstimateEnergyTask(taskCurr),
                        1));
                }
                return err;
            };
        VectorDynamic err = f(x);
        if (H) {
            *H = NumericalDerivativeDynamic(f, x, deltaOptimizer, 1);
            if ((*H)(0, 0) == 0) {
                // int a = 1;
            }
            if (gradientModify != 0) {
                *H = *H * (1 + 0.01 * taskIndex * gradientModify);
            }
        }
        return err;
    }
};

}  // namespace rt_num_opt