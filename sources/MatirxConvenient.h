#pragma once

#include <Eigen/Core>

#include "sources/Tools/testMy.h"
#include "sources/Tools/profilier.h"
namespace rt_num_opt
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
    typedef long long int LLint;
    std::mutex mtx;

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

    inline MatrixDynamic GenerateMatrixDynamic(int m, int n)
    {
        MatrixDynamic M;
        M.resize(m, n);
        M.setZero();
        return M;
    }
    MatrixDynamic GenerateOneMatrix(int m, int n)
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
    std::vector<T> Eigen2Vector(const VectorDynamic &input)
    {
        std::vector<T> res;
        LLint len = input.rows();
        res.reserve(len);
        for (LLint i = 0; i < len; i++)
            res.push_back(input.coeff(i, 0));
        return res;
    }
    template <class T>
    VectorDynamic Vector2Eigen(const std::vector<T> &input)
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

    inline VectorDynamic GenerateVectorDynamic1D(double x)
    {
        VectorDynamic res = GenerateVectorDynamic(1);
        res << x;
        return res;
    }
} // namespace rt_num_opt