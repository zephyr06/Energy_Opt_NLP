#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include <Eigen/Dense>

#include "sources/Tools/colormod.h"

void CoutWarning(std::string message)
{
    std::cout << Color::red << message << Color::def << std::endl;
}
void CoutError(std::string message)
{
    CoutWarning(message);
    throw;
}

/**
 * @brief This function always trigger throw, it just prints information
 *
 * @tparam T
 * @param expect
 * @param actual
 */
template <typename T>
void AssertUnEqual(T expect, T actual, int lineNumber = 0)
{
    if (lineNumber != 0)
        std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
    std::cout << "Assertion failed!" << std::endl;
    std::cout << Color::red << "EXpect is " << expect << ", while the actual is " << actual << Color::def << std::endl;
    throw;
}
void AssertEqualScalar(double expected, double actual, double tolerance = 1e-6, int lineNumber = 0)
{
    if (expected != 0)
    {
        if (abs((expected - actual) / expected) < tolerance)
            return;
        else
        {
            if (lineNumber != 0)
                std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
            AssertUnEqual<double>(expected, actual);
        }
    }
    else
    {
        if (actual != 0)
        {
            if (lineNumber != 0)
                std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
            AssertUnEqual<double>(expected, actual);
        }
    }
}

void AssertBool(bool expected, bool actual, int lineNumber = 0)
{
    if (expected != actual)
        return AssertUnEqual<bool>(expected, actual, lineNumber);
}

template <typename T>
void AssertEqualVectorNoRepeat(const std::vector<T> &expected, const std::vector<T> &actual,
                               double tolerance = 1e-6, int lineNumber = 0)
{
    if (expected.size() != actual.size())
    {
        std::cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }
    // size_t N = expected.size();
    std::unordered_set<T> s;
    for (size_t i = 0; i < expected.size(); i++)
        s.insert(expected.at(i));
    for (size_t i = 0; i < expected.size(); i++)
    {
        if (s.find(actual.at(i)) == s.end())
        {
            CoutError("Actual element " + to_string(actual.at(i)) + " is not found in expected vector");
        }
        else
        {
            s.erase(actual.at(i));
        }
    }

    return;
}

template <typename T>
void AssertEqualVectorExact(const std::vector<T> &expected, const std::vector<T> &actual,
                            double tolerance = 1e-6, int lineNumber = 0)
{
    if (expected.size() != actual.size())
    {
        std::cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }

    for (size_t i = 0; i < expected.size(); i++)
    {
        if (expected[i] != actual[i])
        {
            CoutError("Expected element: " + std::to_string(expected.at(i)) +
                      " while actual element: " + std::to_string(actual.at(i)));
        }
    }

    return;
}

void AssertEigenEqualVector(Eigen::Matrix<double, Eigen::Dynamic, 1> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, 1> actual, int lineNumber = 0)
{
    int m = expected.rows();
    int n = expected.cols();
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j), 1e-6, lineNumber);
        }
    }
}

void AssertEigenEqualMatrix(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> actual, int lineNumber = 0)
{
    int m = expected.rows();
    int n = expected.cols();
    AssertEqualScalar(m, actual.rows(), 1e-6, lineNumber);
    AssertEqualScalar(n, actual.cols(), 1e-6, lineNumber);
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j), 1e-5, lineNumber);
        }
    }
}

template <class T1, class T2>
void AssertEqualMap(std::unordered_map<T1, T2> &mExpect, std::unordered_map<T1, T2> &mActual)
{
    using namespace std;
    if (mExpect.size() != mActual.size())
    {
        CoutWarning("Size error!");
        AssertEqualScalar(mExpect.size(), mActual.size());
    }
    for (auto itr = mExpect.begin(); itr != mExpect.end(); itr++)
    {
        auto itrActual = mActual.find(itr->first);
        if (itrActual == mActual.end() || (itrActual->second).notEqual(itr->second))
        {

            try
            {
                std::cout << "Expect is " << itr->first << ", " << itr->second << endl;
                std::cout << "Actual is " << itrActual->first << ", " << itrActual->second << endl;
            }
            catch (const std::exception &e)
            {
                std::cout << "Cannot print the key to show mismatch element\n";
            }
            CoutError("Element in mExpect is not found in or not equal to mActual!");
        }
    }
}
