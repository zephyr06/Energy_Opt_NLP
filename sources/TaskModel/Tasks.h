#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
// #include <filesystem>
#include <boost/integer/common_factor.hpp>

#include <CppUnitLite/TestHarness.h>

#include "sources/Tools/colormod.h"

#include "sources/Utils/Parameters.h"
#include "sources/MatirxConvenient.h"

namespace rt_num_opt
{

    typedef std::map<int, std::vector<int>> ProcessorTaskSet;
    Color::Modifier red(Color::FG_RED);
    Color::Modifier green(Color::FG_GREEN);
    Color::Modifier blue(Color::FG_BLUE);
    Color::Modifier def(Color::FG_DEFAULT);

    /**
 * @brief generate a random number within the range [a,b];
 * a must be smaller than b
 *
 * @param a
 * @param b
 * @return double
 */
    double RandRange(double a, double b)
    {
        if (b < a)
            CoutError("Range Error in RandRange");
        return a + (b - a) * double(rand()) / RAND_MAX;
    }

    class Task
    {
    public:
        // Member list
        int offset;
        double period;
        int overhead;
        double executionTimeOrg;
        double executionTime;
        int deadline;
        int id;
        int processorId;
        double periodOrg;

        // initializer

        Task() : offset(0), period(0),
                 overhead(0), executionTime(0.0),
                 deadline(0)
        {
            executionTimeOrg = executionTime;
            periodOrg = period;
        }
        Task(int offset, int period, int overhead, double executionTime,
             int deadline) : offset(offset), period(period),
                             overhead(overhead), executionTime(executionTime),
                             deadline(deadline)
        {
            id = -1;
            processorId = -1;
            executionTimeOrg = executionTime;
            periodOrg = period;
        }
        Task(int offset, int period, int overhead, double executionTime,
             int deadline, int id, int processorId) : offset(offset), period(period),
                                                      overhead(overhead), executionTime(executionTime),
                                                      deadline(deadline), id(id),
                                                      processorId(processorId)
        {
            executionTimeOrg = executionTime;
            periodOrg = period;
        }

        /**
     * only used in ReadTaskSet because the input parameter's type is int
     **/
        Task(std::vector<double> dataInLine)
        {
            if (dataInLine.size() < 7)
            {
                std::cout << Color::red << "The length of dataInLine in Task constructor is wrong! Must be at least 7!\n"
                          << Color::def << std::endl;
                throw;
            }
            id = dataInLine[0];
            offset = dataInLine[1];
            period = dataInLine[2];
            overhead = dataInLine[3];
            executionTime = dataInLine[4];
            deadline = dataInLine[5];
            processorId = dataInLine[6];
            executionTimeOrg = executionTime;
        }

        void print()
        {
            std::cout << "The period is: " << period << " The executionTime is " << executionTime << " The deadline is "
                      << deadline << " The overhead is " << overhead << " The offset is " << offset << std::endl;
        }

        double utilization() const
        {
            return double(executionTime) / period;
        }

        int slack()
        {
            return deadline - executionTime;
        }
    };

    typedef std::vector<rt_num_opt::Task> TaskSet;

    class TaskSetNormal
    {
    public:
        TaskSet tasks_;
        int N;
        TaskSetNormal()
        {
            ;
        }
        TaskSetNormal(const TaskSet &tasks) : tasks_(tasks), N(tasks.size()) {}
        static inline std::string Type() { return "normal"; }
        void UpdateTaskSet(TaskSet &tasks)
        {
            tasks_ = tasks;
            N = tasks.size();
        }
    };
    void Print(TaskSet &tasks)
    {
        std::cout << "The task set is printed as follows" << std::endl;
        for (auto &task : tasks)
            task.print();
    }

    template <typename T>
    std::vector<T> GetParameter(const TaskSet &taskset, std::string parameterType)
    {
        uint N = taskset.size();
        std::vector<T> parameterList;
        parameterList.reserve(N);

        for (uint i = 0; i < N; i++)
        {
            if (parameterType == "period")
                parameterList.push_back((T)(taskset[i].period));
            else if (parameterType == "executionTime")
                parameterList.push_back((T)(taskset[i].executionTime));
            else if (parameterType == "overhead")
                parameterList.push_back((T)(taskset[i].overhead));
            else if (parameterType == "deadline")
                parameterList.push_back((T)(taskset[i].deadline));
            else if (parameterType == "offset")
                parameterList.push_back((T)(taskset[i].offset));
            else
            {
                std::cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                          << Color::def << std::endl;
                throw;
            }
        }
        return parameterList;
    }
    template <typename T>
    VectorDynamic GetParameterVD(const TaskSet &taskset, std::string parameterType)
    {
        uint N = taskset.size();
        VectorDynamic parameterList;
        parameterList.resize(N, 1);
        parameterList.setZero();

        for (uint i = 0; i < N; i++)
        {
            if (parameterType == "period")
                parameterList(i, 0) = ((T)(taskset[i].period));
            else if (parameterType == "executionTime")
                parameterList(i, 0) = ((T)(taskset[i].executionTime));
            else if (parameterType == "executionTimeOrg")
                parameterList(i, 0) = ((T)(taskset[i].executionTimeOrg));
            else if (parameterType == "overhead")
                parameterList(i, 0) = ((T)(taskset[i].overhead));
            else if (parameterType == "deadline")
                parameterList(i, 0) = ((T)(taskset[i].deadline));
            else if (parameterType == "offset")
                parameterList(i, 0) = ((T)(taskset[i].offset));
            else
            {
                std::cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                          << Color::def << std::endl;
                throw;
            }
        }
        return parameterList;
    }

    template <typename T>
    VectorDynamic GetParameterVD(const TaskSetNormal &taskset, std::string parameterType)
    {
        return GetParameterVD<T>(taskset.tasks_, parameterType);
    }
    // some helper function for Reorder
    static bool comparePeriod(Task task1, Task task2)
    {
        return task1.period < task2.period;
    };
    bool compareUtilization(Task task1, Task task2)
    {
        return task1.utilization() < task2.utilization();
    };
    static bool compareDeadline(Task &task1, Task &task2)
    {
        return task1.deadline < task2.deadline;
    };

    double Utilization(const TaskSet &tasks)
    {
        std::vector<int> periodHigh = GetParameter<int>(tasks, "period");
        std::vector<double> executionTimeHigh = GetParameter<double>(tasks, "executionTime");
        int N = periodHigh.size();
        double utilization = 0;
        for (int i = 0; i < N; i++)
            utilization += double(executionTimeHigh[i]) / periodHigh[i];
        return utilization;
    }

    // Recursive function to return gcd of a and b
    long long gcd(long long int a, long long int b)
    {
        if (b == 0)
            return a;
        return gcd(b, a % b);
    }

    // Function to return LCM of two numbers
    long long int lcm(long long int a, long long int b)
    {
        return (a / gcd(a, b)) * b;
    }

    long long int HyperPeriod(const TaskSet &tasks)
    {
        int N = tasks.size();
        if (N == 0)
        {
            std::cout << Color::red << "Empty task set in HyperPeriod()!\n";
            throw;
        }
        else
        {
            long long int hyper = ceil(tasks[0].period);
            for (int i = 1; i < N; i++)
            {
                hyper = lcm(hyper, int(ceil(tasks[i].period)));
                if (hyper < 0 || hyper > LLONG_MAX)
                {
                    // cout << Color::red <<  << Color::def << std::endl;
                    // throw;
                    if (debugMode == 1)
                        CoutWarning("The hyper-period over flows!");
                    return LLONG_MAX / 2;
                }
            }
            return hyper;
        }
    }

    TaskSet Reorder(TaskSet tasks, std::string priorityType)
    {
        if (priorityType == "RM")
        {
            sort(tasks.begin(), tasks.end(), comparePeriod);
        }
        else if (priorityType == "utilization")
        {
            sort(tasks.begin(), tasks.end(), compareUtilization);
        }
        else if (priorityType == "deadline")
        {
            sort(tasks.begin(), tasks.end(), compareDeadline);
        }
        else if (priorityType == "orig")
        {
            ;
        }
        else
        {
            std::cout << Color::red << "Unrecognized priorityType in Reorder!\n"
                      << Color::def << std::endl;
            throw;
        }
        return tasks;
    }

    /**
 * @brief read from WriteFrequencyModelRatio()
 *
 * @param path
 */
    void ReadFrequencyModeRatio(std::string path)
    {

        std::fstream file;
        file.open(path, std::ios::in);
        if (file.is_open())
        {
            std::string line;
            while (getline(file, line))
            {
                if (line.substr(0, 17) == "Frequency_Ratio: ")
                {
                    frequencyRatio = std::stod(line.substr(17));
                    if (frequencyRatio == 0)
                        frequencyRatio = 0.5;
                    return;
                }
            }
            if (frequencyRatio == 0)
                frequencyRatio = 0.5;
        }
        else
        {
            CoutError("The path does not exist in ReadFrequencyModeRatio! Given path is " + path);
        }
    }

    std::vector<std::string> SplitStringMy(std::string line, std::string delimiter = ",")
    {
        // some default parameters in this function
        std::string token;
        size_t pos = 0;
        std::vector<std::string> dataInLine;
        while ((pos = line.find(delimiter)) != std::string::npos)
        {
            token = line.substr(0, pos);
            dataInLine.push_back(token);
            line.erase(0, pos + delimiter.length());
        }
        dataInLine.push_back((line.c_str()));
        return dataInLine;
    }

    std::vector<double> ReadLine(std::string line, std::string delimiter = ",")
    {
        // some default parameters in this function
        using namespace std;
        string token;
        size_t pos = 0;
        vector<double> dataInLine;
        while ((pos = line.find(delimiter)) != string::npos)
        {
            token = line.substr(0, pos);
            double temp = stod(token.c_str());
            dataInLine.push_back(temp);
            line.erase(0, pos + delimiter.length());
        }
        if (line.find_first_not_of(' ') != std::string::npos)
            dataInLine.push_back(stod(line.c_str()));
        return dataInLine;
    }
    TaskSet ReadTaskSet(std::string path, std::string priorityType = "RM")
    {
        using namespace std;
        ReadFrequencyModeRatio(path);

        std::vector<Task> taskSet;

        fstream file;
        file.open(path, ios::in);
        if (file.is_open())
        {
            string line;
            while (getline(file, line))
            {
                if (!(line[0] >= '0' && line[0] <= '9'))
                    continue;
                vector<double> dataInLine = ReadLine(line);

                // dataInLine.erase(dataInLine.begin());
                Task taskCurr(dataInLine);
                taskSet.push_back(taskCurr);
            }

            TaskSet ttt(taskSet);
            ttt = Reorder(ttt, priorityType);
            if (debugMode == 1)
                std::cout << "Finish reading the data file succesfully!\n";
            return ttt;
        }
        else
        {
            std::cout << Color::red << "The path does not exist in ReadTaskSet!" << std::endl
                      << path
                      << Color::def << std::endl;
            throw;
        }
    }

    void UpdateTaskSetExecutionTime(TaskSet &taskSet, VectorDynamic executionTimeVec, int lastTaskDoNotNeedOptimize = -1)
    {
        int N = taskSet.size();

        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            taskSet[i].executionTime = executionTimeVec(i - lastTaskDoNotNeedOptimize - 1, 0);
    }

    void UpdateTaskSetPeriod(TaskSet &taskSet, VectorDynamic periodVec, int lastTaskDoNotNeedOptimize = -1)
    {
        int N = taskSet.size();

        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            taskSet[i].period = periodVec(i - lastTaskDoNotNeedOptimize - 1, 0);
    }
    ProcessorTaskSet ExtractProcessorTaskSet(TaskSet &tasks)
    {
        int N = tasks.size();
        ProcessorTaskSet processorTasks;
        for (int i = 0; i < N; i++)
        {
            if (processorTasks.find(tasks[i].processorId) == processorTasks.end())
            {
                std::vector<int> ttt{tasks[i].id};
                processorTasks[tasks[i].processorId] = ttt;
            }
            else
            {
                processorTasks[tasks[i].processorId].push_back(tasks[i].id);
            }
        }
        return processorTasks;
    }
} // namespace rt_num_opt