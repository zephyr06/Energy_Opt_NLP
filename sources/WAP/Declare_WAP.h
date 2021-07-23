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
#include <gtsam/base/Testable.h>

#include "../Parameters.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> SquareMatrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;

SquareMatrix A_Global;
SquareMatrix P_Global;

SquareMatrix GenerateZeroMatrix(int N)
{
    SquareMatrix A;
    A.resize(N, N);
    A.setZero();
    return A;
}
SquareMatrix GenerateOneMatrix(int N)
{
    SquareMatrix A;
    A.resize(N, N);
    A.setZero();
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            A(i, j) = 1;
        }
    }
    return A;
}
