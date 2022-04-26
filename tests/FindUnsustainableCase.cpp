#include <CppUnitLite/TestHarness.h>

#include "sources/RTA/RTA_Fonseca2019.h"
#include "sources/BatchTestutils.h"

TEST(batchfind, v1)
{
    const char *pathDataset;
    pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/";

    std::vector<double> energySaveRatioVec;
    std::vector<double> runTime;
    int N;
    if (rt_num_opt::debugMode == 1)
        printf("Directory: %s\n", pathDataset);
    std::vector<std::string> errorFiles;
    for (const auto &file : rt_num_opt::ReadFilesInDirectory(pathDataset))
    {
        if (rt_num_opt::debugMode)
            std::cout << file << std::endl;
        std::string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "periodic")
        {
            std::string path = pathDataset + file;
            rt_num_opt::DAG_Model dagTasks = rt_num_opt::ReadDAG_Task(path, rt_num_opt::readTaskMode);
            auto tasksetVerucchi = TransformTaskSetNumOpt2dagSched(dagTasks);
            double rtaBase = dagSched::RTA_Fonseca2019(tasksetVerucchi, 4)[0];
            for (size_t i = 0; i < dagTasks.N; i++)
            {
                for (int j = 0; j < 100000; j++)
                {
                    dagTasks.tasks_[i].executionTime += 1;
                    if (dagTasks.tasks_[i].executionTime > dagTasks.tasks_[i].executionTimeOrg * 3)
                    {
                        break;
                    }
                    tasksetVerucchi = TransformTaskSetNumOpt2dagSched(dagTasks);
                    double rta = dagSched::RTA_Fonseca2019(tasksetVerucchi, 8)[0];
                    if (rta < rtaBase)
                    {
                        std::cout << "Find one un-sustainable task config" << std::endl;
                        std::cout << path << std::endl;
                        std::cout << "Task index: " << i << std::endl;
                        std::cout << "Task's executionTime is " << dagTasks.tasks_[i].executionTime << std::endl;
                        std::cout << "Rta difference: " << rtaBase << ", " << rta << std::endl;
                    }
                }
                dagTasks.tasks_[i].executionTime = dagTasks.tasks_[i].executionTimeOrg;
            }
        }
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
