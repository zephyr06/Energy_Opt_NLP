#pragma once

#include <boost/function.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "testMy.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;

double min(double a, double b)
{
    if (a <= b)
        return a;
    else
        return b;
    return 0;
}
double max(double a, double b)
{
    if (a >= b)
        return a;
    else
        return b;
    return 0;
}
// typedef Eigen::Matrix<double, TASK_NUMBER, 1> ComputationTimeVector;
// typedef Eigen::Matrix<double, TASK_NUMBER, TASK_NUMBER> JacobianOpt;
// typedef Eigen::Matrix<double, TASK_NUMBER, 1> ErrElement;