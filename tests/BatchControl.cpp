#include "sources/BatchControlOptimize.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
int main(int argc, char *argv[])
{
    BeginTimer(__func__);
    if (argc == 1)
        BatchOptimize<FactorGraphInManifold>();
    else if (argc == 2)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            std::cout << "Task sets under analyzation is N" + std::to_string(N) << std::endl;
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
            std::cout << "Task sets under analyzation is N" + std::to_string(N) << std::endl;
        if (N >= 0)
            BatchOptimize<FactorGraphForceManifold>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
    }
    else
    {
        CoutError("Too many arguments in LLBatch!");
    }
    EndTimer(__func__);
    PrintTimer();
}
