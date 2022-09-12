#include <CppUnitLite/TestHarness.h>

#include "sources/RTA/RTA_Nasri19.h"
#include "sources/BatchTestutils.h"
#include "sources/Tools/colormod.h"

TEST(find, v1)
{
    std::vector<int> Ns = {3, 4, 5, 6, 7, 8, 9, 10};
    for (int N : Ns)
    {
        std::cout << "Checking " << N << std::endl;
        const char *pathDataset;
        std::string str = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" + std::to_string(N) + "/";
        pathDataset = str.c_str();
        for (auto file : rt_num_opt::ReadFilesInDirectory(pathDataset))
        {
            std::string delimiter = "-";
            if (file.substr(0, file.find(delimiter)) == "periodic" && file.substr(file.length() - 4, 4) == "yaml")
            {
                std::string path = pathDataset + file;
                auto tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
                rt_num_opt::RTA_Nasri19 r(tasksN);
                if (!r.CheckSchedulability())
                {
                    std::cout << Color::red << file << std::endl
                              << Color::def;
                }
            }
        }
    }
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
