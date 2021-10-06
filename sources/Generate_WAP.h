#include <tuple>
#include "RTA_WAP.h"

bool AssignLogicWAP(int index, const TaskSet &tasks, const MatrixDynamic &A, const MatrixDynamic &P)
{
    ;
}

tuple<bool, MatrixDynamic, MatrixDynamic> Generate_WAP(const TaskSet &tasks)
{
    int N = tasks.size();
    MatrixDynamic A = GenerateZeroMatrix(N, N);
    MatrixDynamic P = GenerateZeroMatrix(N, N);

    for (int i = 0; i < N - 1; i++)
    {
        if (not AssignLogicWAP(i, tasks, A, P))
        {
            // Not schedulable
            return make_tuple(0, A, P);
        }
    }

    // check the last task
    if (RTA_WAP::)
}