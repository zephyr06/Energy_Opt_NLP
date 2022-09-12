#include <CppUnitLite/TestHarness.h>

#include "sources/RTA/RTA_Nasri19.h"
#include "sources/BatchTestutils.h"
#include "sources/Utils/Parameters.h"

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
    std::vector<std::string> pathTaskSet;
    for (const auto &file : rt_num_opt::ReadFilesInDirectory(pathDataset))
    {
        if (rt_num_opt::debugMode)
            std::cout << file << std::endl;
        std::string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag")
        {
            std::string path = pathDataset + file;
            pathTaskSet.push_back(path);
            if (pathTaskSet.size() < rt_num_opt::taskSetSize_FindUnsustainable)
            {
                continue;
            }
            std::vector<rt_num_opt::DAG_Model> dagTasksNumOpt = rt_num_opt::TransformTaskSetNumOpt2dagSched(pathTaskSet);

            int adjustTaskIndex = 0;
            for (int adjustTaskIndex = 0; adjustTaskIndex < rt_num_opt::taskSetSize_FindUnsustainable; adjustTaskIndex++)
            {
                auto tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(dagTasksNumOpt);

                std::vector<double> rtaBase = dagSched::RTA_G_LP_FTP_Nasri2019_C(tasksetVerucchi, rt_num_opt::core_m_dag);
                if (rtaBase.size() == 0)
                {
                    continue;
                }
                for (size_t i = 0; i < rt_num_opt::taskSetSize_FindUnsustainable; i++)
                {
                    for (int j = 0; j < 100000; j++)
                    {
                        dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTime += 1;
                        if (dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTime > dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTimeOrg * 5)
                        {
                            break;
                        }
                        tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(dagTasksNumOpt);
                        std::vector<double> rta = dagSched::RTA_G_LP_FTP_Nasri2019_C(tasksetVerucchi, rt_num_opt::core_m_dag);
                        bool findOne = rta.size() == 0;
                        for (size_t i = 0; i < rta.size() && findOne == false; i++)
                        {
                            if (rta[i] < rtaBase[i])
                            {
                                findOne = true;
                            }
                        }
                        if (rta.size() == 0 || findOne)
                        {
                            // if(rtaBase==true && rta==false )
                            std::cout << "Find one un-sustainable task config" << std::endl;
                            for (size_t kkk = 0; kkk < pathTaskSet.size(); kkk++)
                            {
                                std::cout << pathTaskSet[kkk] << std::endl;
                            }

                            std::cout << "Task index: " << i << std::endl;
                            std::cout << "Task's executionTime is " << dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTime << std::endl;
                            std::cout << "Rta difference: " << std::endl;
                            for (size_t kkk = 0; kkk < rta.size(); kkk++)
                            {
                                std::cout << rtaBase[kkk] << ", " << rta[kkk] << std::endl;
                            }
                        }
                        else
                        {
                            rtaBase == rta;
                        }
                    }
                    dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTime = dagTasksNumOpt[i].tasks_[adjustTaskIndex].executionTimeOrg;
                }
            }
            pathTaskSet.clear();
        }
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
