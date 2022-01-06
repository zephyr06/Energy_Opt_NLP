#include "Parameters.h"
#include "Declaration.h"
/**
 * barrier function for the optimization
 **/
double Barrier(double x)
{
    if (x > 0)
        // return pow(x, 2);
        return 0;
    else if (x < 0)
    {
        if (TASK_NUMBER == 0)
        {
            cout << red << "Please set TASK_NUMBER!" << def << endl;
            throw;
        }
        return punishmentInBarrier * pow(10, TASK_NUMBER - 3) * pow(-1 * x, 1);
    }
    else // it basically means x=0
        return weightLogBarrier *
               log(x + toleranceBarrier);
}

MatrixDynamic NumericalDerivativeDynamic(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                         VectorDynamic x, double deltaOptimizer, int mOfJacobian)
{
    int n = x.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);

    for (int i = 0; i < n; i++)
    {
        VectorDynamic xDelta = x;
        xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
        VectorDynamic resPlus;
        resPlus.resize(mOfJacobian, 1);
        resPlus = h(xDelta);
        // cout << "resPlus" << endl
        //      << resPlus << endl;

        xDelta(i, 0) = xDelta(i, 0) - 2 * deltaOptimizer;
        VectorDynamic resMinus;
        resMinus.resize(mOfJacobian, 1);
        resMinus = h(xDelta);

        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
        }
    }
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

// ------------------ two convenient functions for ClampComputationTime
double JacobianInClamp(TaskSet &tasks, VectorDynamic &comp, int i)
{
    return 1.0 / tasks[i].period * pow(double(tasks[i].executionTime) / comp(i, 0), 3);
};

// to sort from the biggest to smallest
bool comparePair(const pair<int, double> &p1, const pair<int, double> &p2)
{
    return (p1.second > p2.second);
}

/**
 * @brief whether tasksCurr's computation time is within given bound of tasksRef
 * 
 * @param tasksRef 
 * @param tasksCurr 
 * @return true 
 * @return false 
 */
bool WithInBound(const TaskSet &tasksRef, const TaskSet &tasksCurr)
{
    int N = tasksRef.size();
    for (int i = 0; i < N; i++)
    {
        if (tasksRef[i].executionTime * 2 < tasksCurr[i].executionTime)
            return false;
        if (tasksRef[i].executionTime > tasksCurr[i].executionTime)
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
