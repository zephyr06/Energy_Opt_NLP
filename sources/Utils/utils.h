#pragma once

#include <boost/function.hpp>

#include "sources/Utils/Parameters.h"
#include "sources/MatrixConvenient.h"
#include "sources/TaskModel/Tasks.h"

namespace rt_num_opt
{

    /**
     * barrier function for the optimization
     **/
    double Barrier(double x)
    {

        if (x >= 0)
        {
            if (whether_IPM)
                return std::log(x);
            else
                return 0;
        }

        else //(x < 0)
        {
            return punishmentInBarrier * pow(-1 * x, 1);
        }
    }

    MatrixDynamic NumericalDerivativeDynamic(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                             VectorDynamic x, double deltaOptimizer, int mOfJacobian = -1)
    {
        BeginTimer(__func__);
        int n = x.rows();
        MatrixDynamic jacobian;
        if (mOfJacobian == -1)
            mOfJacobian = h(x).rows();
        jacobian.resize(mOfJacobian, n);

        for (int i = 0; i < n; i++)
        {
            VectorDynamic xDelta = x;
            xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
            VectorDynamic resPlus;
            resPlus.resize(mOfJacobian, 1);
            resPlus = h(xDelta);
            // cout << "resPlus" << std::endl
            //      << resPlus << std::endl;

            xDelta(i, 0) = xDelta(i, 0) - 2 * deltaOptimizer;
            VectorDynamic resMinus;
            resMinus.resize(mOfJacobian, 1);
            resMinus = h(xDelta);

            for (int j = 0; j < mOfJacobian; j++)
            {
                jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
            }
        }
        EndTimer(__func__);
        return jacobian;
    }

    MatrixDynamic NumericalDerivativeDynamicUpper(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                                  VectorDynamic x, double deltaOptimizer, int mOfJacobian)
    {
        int n = x.rows();
        MatrixDynamic jacobian;
        jacobian.resize(mOfJacobian, n);
        VectorDynamic currErr = h(x);

        for (int i = 0; i < n; i++)
        {
            VectorDynamic xDelta = x;
            xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
            VectorDynamic resPlus;
            resPlus.resize(mOfJacobian, 1);
            resPlus = h(xDelta);
            for (int j = 0; j < mOfJacobian; j++)
            {
                jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
            }
        }
        return jacobian;
    }

    // ------------------  convenient function for ClampComputationTime

    // to sort from the  smallest to biggest (minimum negative gradient first)
    bool comparePair(const std::pair<int, double> &p1, const std::pair<int, double> &p2)
    {
        return (p1.second < p2.second);
    }

    /**
     * @brief whether tasksCurr's computation time is within given bound of tasksRef
     *
     * @param tasksRef
     * @param tasksCurr
     * @return true
     * @return false
     */
    bool WithInBound(const TaskSet &tasks)
    {
        if (not enableMaxComputationTimeRestrict)
            return true;
        int N = tasks.size();
        for (int i = 0; i < N; i++)
        {
            if (tasks[i].executionTimeOrg * MaxComputationTimeRestrict < tasks[i].executionTime)
                return false;
            else if (tasks[i].executionTimeOrg > tasks[i].executionTime)
                return false;
        }
        return true;
    }

    /**
     * for minimization problem
     */
    bool checkConvergenceInterior(double oldY, VectorDynamic oldX, double newY, VectorDynamic newX,
                                  double relativeErrorTol, double xTol)
    {
        double relDiff = (oldY - newY) / oldY;
        double xDiff = (oldX - newX).norm();
        if (relDiff < relErrorTolIPM || xDiff < relativeErrorTol)
            return true;

        else
            return false;
    }

} // namespace rt_num_opt
