#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TYPE_A;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TYPE_P;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
