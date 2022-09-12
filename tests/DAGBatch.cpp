#include "sources/BatchOptimize.h"
#include "sources/RTA/RTA_Nasri19.h"
using namespace rt_num_opt;
int main(int argc, char *argv[])
{
    if (argc == 1)
        BatchOptimize<DAG_Nasri19, RTA_Nasri19>();
    else if (argc == 2)
    {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        if (debugMode == 1)
            std::cout << "Task sets under analyzation is N" + std::to_string(N) << std::endl;
        if (N >= 0)
            BatchOptimize<DAG_Nasri19, RTA_Nasri19>(N);
        else
            CoutError("Unrecognized arguments in LLBatch!");
        PrintTimer();
    }
    else
    {
        CoutError("Too many arguments in LLBatch!");
    }
}
