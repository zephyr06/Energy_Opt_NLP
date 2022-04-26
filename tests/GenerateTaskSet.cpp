#include <filesystem>
#include "sources/TaskModel/GenerateRandomTaskset.h"
#include "sources/argparse.hpp"
#include "sources/RTA/RTA_LL.h"
#include "sources/RTA/RTA_DAG.h"
using namespace rt_num_opt;
using namespace std;
void deleteDirectoryContents(const std::string &dir_path)
{

    namespace fs = std::filesystem;
    if (!fs::is_directory(dir_path) || !fs::exists(dir_path))
    {                                   // Check if dir_path folder exists
        fs::create_directory(dir_path); // create dir_path folder
    }

    for (const auto &entry : std::filesystem::directory_iterator(dir_path))
        std::filesystem::remove_all(entry.path());
}
int main(int argc, char *argv[])
{
    argparse::ArgumentParser program("program name");
    program.add_argument("-v", "--verbose"); // parameter packing

    program.add_argument("--N")
        .default_value(5)
        .help("the number of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--taskSetNumber")
        .default_value(10)
        .help("the number DAGs to create")
        .scan<'i', int>();
    program.add_argument("--totalUtilization")
        .default_value(0.4)
        .help("the total utilization of tasks in each DAG")
        .scan<'f', double>();
    program.add_argument("--NumberOfProcessor")
        .default_value(2)
        .help("the NumberOfProcessor of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--periodMin")
        .default_value(100)
        .help("the minimum period of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--periodMax")
        .default_value(500)
        .help("the maximum period of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--deadlineType")
        .default_value(0)
        .help("type of tasksets, 0 means implicit, 1 means random")
        .scan<'i', int>();
    program.add_argument("--taskType")
        .default_value(1)
        .help("type of tasksets, 0 means normal, 1 means DAG")
        .scan<'i', int>();
    program.add_argument("--schedulabilityCheck")
        .default_value(0)
        .help("schedulabilityCheck in generate task sets? If so, all the task sets are schedulable")
        .scan<'i', int>();
    // program.add_argument("--parallelismFactor")
    //     .default_value(1000)
    //     .help("the parallelismFactor DAG")
    //     .scan<'i', int>();

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    int N = program.get<int>("--N");
    int DAG_taskSetNumber = program.get<int>("--taskSetNumber");
    double totalUtilization = program.get<double>("--totalUtilization");
    int numberOfProcessor = program.get<int>("--NumberOfProcessor");
    int periodMin = program.get<int>("--periodMin");
    int periodMax = program.get<int>("--periodMax");
    int deadlineType = program.get<int>("--deadlineType");
    int taskType = program.get<int>("--taskType");
    int schedulabilityCheck = program.get<int>("--schedulabilityCheck");
    cout << "Task configuration: " << endl
         << "the number of tasks(--N): " << N << endl
         << "taskSetNumber(--taskSetNumber): " << DAG_taskSetNumber << endl
         << "totalUtilization(--totalUtilization): " << totalUtilization << endl
         << "NumberOfProcessor(--NumberOfProcessor): " << numberOfProcessor << endl
         << "periodMin(--periodMin): " << periodMin << endl
         << "periodMax(--periodMax): " << periodMax << endl
         << "deadlineType(--deadlineType), 1 means random, 0 means implicit: " << deadlineType << endl
         << "taskType(--taskType), type of tasksets, 0 means normal, 1 means DAG: " << taskType << endl
         << "schedulabilityCheck(--schedulabilityCheck), 0 means no, 1 means yes via LL, 2 means yes via DAG: " << schedulabilityCheck << endl
         << endl;

    string outDirectory;
    if (schedulabilityCheck)
    {
        outDirectory = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" + to_string(N) + "/";
    }
    else
    {
        outDirectory = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/";
    }

    deleteDirectoryContents(outDirectory);
    srand(time(0));
    for (size_t i = 0; i < DAG_taskSetNumber; i++)
    {
        if (taskType == 0)
        {
            TaskSet tasks = GenerateTaskSet(N, totalUtilization,
                                            numberOfProcessor,
                                            periodMin,
                                            periodMax, deadlineType);
            TaskSetNormal tasksN(tasks);
            if (schedulabilityCheck)
            {
                if (schedulabilityCheck == 1)
                {
                    RTA_LL r(tasksN);
                    if (!r.CheckSchedulability())
                    {
                        i--;
                        continue;
                    }
                }
            }
            string fileName = "periodic-set-" + string(3 - to_string(i).size(), '0') + to_string(i) + "-syntheticJobs" + ".csv";

            ofstream myfile;
            myfile.open(outDirectory + fileName);
            WriteTaskSets(myfile, tasks);
        }
        else if (taskType == 1)
        {
            std::vector<DAG_Model> dagTaskSet;
            vector<double> utilVec = Uunifast(N, totalUtilization);
            vector<double> weightVec;
            for (int i = 0; i < N; i++)
            {
                int periodCurr = RandRange(periodMin, periodMax);
                dagTaskSet.push_back(GenerateDAG(ceil(RandRange(1, 2 * N)), utilVec[i],
                                                 numberOfProcessor,
                                                 periodCurr,
                                                 periodCurr, deadlineType));
                weightVec.push_back(RandRange(1e-3, 1000));
            }

            string fileName = "periodic-dag-Melani-set-" + string(3 - to_string(i).size(), '0') + to_string(i) + "-syntheticJobs" + ".csv";
            ofstream myfile;
            myfile.open(outDirectory + fileName);
            WriteDAGMelani(myfile, dagTaskSet, weightVec);
            myfile.close();
            if (schedulabilityCheck)
            {
                if (schedulabilityCheck == 2)
                {
                    TaskSetDAG tasksDAG = ReadDAG_Tasks(outDirectory + fileName, readTaskMode);
                    RTA_DAG r(tasksDAG);
                    if (!r.CheckSchedulability())
                    {
                        i--;
                        continue;
                    }
                }
            }
        }
    }

    return 0;
}