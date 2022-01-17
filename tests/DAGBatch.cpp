#include "../sources/BatchOptimize.h"

int main(int argc, char *argv[])
{
    if (argc == 1)
        BatchOptimize<TaskSetDAG, RTA_DAG>();
    else if (argc == 2)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            cout << "Task sets under analyzation is N" + to_string(N) << endl;
        if (N >= 0)
            BatchOptimize<TaskSetDAG, RTA_DAG>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
    }
    else
    {
        CoutError("Too many arguments in LLBatch!");
    }
}
