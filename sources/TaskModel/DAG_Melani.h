#include "sources/TaskModel/DAG_Task.h"
namespace rt_num_opt
{
    //************Melani_DAG******************** //
    struct TaskSetDAG : public TaskSetNormal
    {
    public:
        std::vector<double> volumeVec_;
        std::vector<double> longestVec_;
        std::vector<double> weightVec_;

        TaskSetDAG() : TaskSetNormal() {}
        TaskSetDAG(const TaskSet &tasks,
                   std::vector<double> &volumeVec,
                   std::vector<double> &longestVec, std::vector<double> &weight)
            : TaskSetNormal(tasks), volumeVec_(volumeVec),
              longestVec_(longestVec), weightVec_(weight) {}
        static std::string Type() { return "dag"; }
    };

    TaskSetDAG ReadDAG_Tasks(std::string path, std::string priorityType = "orig")
    {
        ReadFrequencyModeRatio(path);

        std::vector<Task> taskSet;
        std::vector<double> volumeVec;
        std::vector<double> longestVec;
        std::vector<double> weightVec;

        std::fstream file;
        file.open(path, std::ios::in);
        if (file.is_open())
        {
            std::string line;

            while (getline(file, line))
            {
                if (!(line[0] >= '0' && line[0] <= '9'))
                    continue;
                std::vector<double> dataInLine = ReadLine(line);
                if (dataInLine.size() < 9)
                    CoutError("The path in ReadDAG_Tasks doesn't follow Melani format!");
                weightVec.push_back(dataInLine.back());
                dataInLine.pop_back();
                longestVec.push_back(dataInLine.back());
                dataInLine.pop_back();
                volumeVec.push_back(dataInLine.back());
                dataInLine.pop_back();

                Task taskCurr(dataInLine);
                taskSet.push_back(taskCurr);
            }
            TaskSet tasks(taskSet);
            tasks = Reorder(tasks, priorityType);
            TaskSetDAG dagTaskSets(tasks, volumeVec, longestVec, weightVec);
            if (debugMode == 1)
                std::cout << "Finish reading the data file succesfully!\n";
            return dagTaskSets;
        }
        else
        {
            std::cout << Color::red << "The path does not exist in ReadTaskSet!" << std::endl
                      << path
                      << Color::def << std::endl;
            throw;
        }
    }
} // namespace rt_num_opt