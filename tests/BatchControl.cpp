#include "../sources/BatchControlOptimize.h"

int main(int argc, char *argv[])
{
    if (argc == 1)
        BatchOptimize<FactorGraphInManifold>();
    else if (argc == 2)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            cout << "Task sets under analyzation is N" + to_string(N) << endl;
        if (N >= 0)
            BatchOptimize<FactorGraphInManifold>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
    }
    else if (argc == 3)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            cout << "Task sets under analyzation is N" + to_string(N) << endl;
        if (N >= 0)
            BatchOptimize<FactorGraphForceManifold>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
    }
    else
    {
        CoutError("Too many arguments in LLBatch!");
    }
}
