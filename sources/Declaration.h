#pragma once

#include <boost/function.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "testMy.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;
// extern MatrixDynamic A_Global;
// extern MatrixDynamic P_Global;
MatrixDynamic A_Global;
MatrixDynamic P_Global;

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

inline MatrixDynamic GenerateZeroMatrix(int m, int n)
{
    MatrixDynamic M;
    M.resize(m, n);
    M.setZero();
    return M;
}
inline MatrixDynamic GenerateOneMatrix(int m, int n)
{
    MatrixDynamic M;
    M.resize(m, n);
    M.setZero();
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
            M(i, j) = 1;
    }
    return M;
}
template <class T>
vector<T> Eigen2Vector(const VectorDynamic &input)
{
    vector<T> res;
    LLint len = input.rows();
    res.reserve(len);
    for (LLint i = 0; i < len; i++)
        res.push_back(input.coeff(i, 0));
    return res;
}
template <class T>
VectorDynamic Vector2Eigen(const vector<T> &input)
{

    LLint len = input.size();
    VectorDynamic res;
    res.resize(len, 1);
    for (LLint i = 0; i < len; i++)
        res(i, 0) = input.at(i);
    return res;
}
inline VectorDynamic GenerateVectorDynamic(LLint N)
{
    VectorDynamic v;
    v.resize(N, 1);
    v.setZero();
    return v;
}