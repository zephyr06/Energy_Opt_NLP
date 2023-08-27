#include "sources/ControlOptimization/BatchControlOptimizeNasri19.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
int main(int argc, char *argv[]) {
    if (argc == 1)
        CoutError("Need 2 args!");
    else if (argc == 2) {
        char *pEnd;
        int N = strtol(argv[1], &pEnd, 10);
        const char *pathDataset;
        std::string str = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" +
                          batchOptimizeFolder + "/N" + std::to_string(N) + "/";
        pathDataset = str.c_str();

        printf("Directory: %s\n", pathDataset);
        std::vector<double> util_vec;
        for (const auto &file : ReadFilesInDirectory(pathDataset)) {
            int type = ControlNasri19::TargetFileType(file);
            if (type == 0) {
                std::string path = pathDataset + file;
                std::cout << path << "\n";
                DAG_Nasri19 dag_tasks = ReadDAGNasri19_Tasks(path);
                util_vec.push_back(Utilization(dag_tasks.tasks_));
            }
        }
        std::cout << "Average utilization: " << Average(util_vec) << "\n";

    } else {
        CoutError("Too many arguments in LLBatch!");
    }
}
