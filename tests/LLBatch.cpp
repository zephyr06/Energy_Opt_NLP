#include "sources/BatchOptimize.h"
using namespace rt_num_opt;
int main(int argc, char *argv[])
{
    runMode == "normal";
    if (argc == 1)
        BatchOptimize<TaskSetNormal, RTA_LL>();
    else if (argc == 2)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            cout << "Task sets under analyzation is N" + to_string(N) << endl;
        if (N >= 0)
            BatchOptimize<TaskSetNormal, RTA_LL>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
    }
    else
    {
        CoutError("Too many arguments in LLBatch!");
    }
}
