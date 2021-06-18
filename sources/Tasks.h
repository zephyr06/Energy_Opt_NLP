#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
#include <filesystem>
#include <boost/integer/common_factor.hpp>

#include <CppUnitLite/TestHarness.h>

using namespace std;

#define TaskSet vector<Task>

class Task
{
public:
    // Member list
    int offset;
    int period;
    int overhead;
    int executionTime;
    int deadline;

    // initializer

    Task() : offset(0), period(0),
             overhead(0), executionTime(0),
             deadline(0) {}
    Task(int offset, int period, int overhead, int executionTime,
         int deadline) : offset(offset), period(period),
                         overhead(overhead), executionTime(executionTime),
                         deadline(deadline) {}

    Task(vector<int> dataInLine)
    {
        if (dataInLine.size() != 5)
        {
            cout << "The length of dataInLine in Task constructor is wrong! Must be 5!\n";
            throw;
        }

        offset = dataInLine[0];
        period = dataInLine[1];
        overhead = dataInLine[2];
        executionTime = dataInLine[3];
        deadline = dataInLine[4];
    }

    void print()
    {
        cout << "The period is: " << period << " The executionTime is " << executionTime << " The deadline is "
             << deadline << " The overhead is " << overhead << " The offset is " << offset << endl;
    }

    float utilization()
    {
        return float(executionTime) / period;
    }

    int slack()
    {
        return deadline - executionTime;
    }
};

vector<int> GetParameter(const TaskSet &taskset, string parameterType)
{
    uint N = taskset.size();
    vector<int> parameterList;
    parameterList.reserve(N);

    for (uint i = 0; i < N; i++)
    {
        if (parameterType == "period")
            parameterList.push_back(taskset[i].period);
        else if (parameterType == "executionTime")
            parameterList.push_back(taskset[i].executionTime);
        else if (parameterType == "overhead")
            parameterList.push_back(taskset[i].overhead);
        else if (parameterType == "deadline")
            parameterList.push_back(taskset[i].deadline);
        else if (parameterType == "offset")
            parameterList.push_back(taskset[i].offset);
        else
        {
            cout << "parameterType in GetParameter is not recognized!\n";
            throw;
        }
    }
    return parameterList;
}

// some helper function for Reorder
static bool comparePeriod(Task task1, Task task2)
{
    return task1.period < task2.period;
};

float Utilization(const TaskSet &tasks)
{
    vector<int> periodHigh = GetParameter(tasks, "period");
    vector<int> executionTimeHigh = GetParameter(tasks, "executionTime");
    int N = periodHigh.size();
    float utilization = 0;
    for (int i = 0; i < N; i++)
        utilization += float(executionTimeHigh[i]) / periodHigh[i];
    return utilization;
}

int HyperPeriod(const TaskSet &tasks)
{
    int N = tasks.size();
    if (N == 0)
    {
        cout << "Empty task set in HyperPeriod()!\n";
        throw;
    }
    else
    {
        int hyper = tasks[0].period;
        for (int i = 1; i < N; i++)
            hyper = boost::integer::lcm(hyper, tasks[i].period);
        return hyper;
    }
}

TaskSet Reorder(TaskSet tasks, string priorityType)
{
    if (priorityType == "RM")
    {
        sort(tasks.begin(), tasks.end(), comparePeriod);
    }
    else if (priorityType == "orig")
    {
        ;
    }
    else
    {
        cout << "Unrecognized priorityType in Reorder!\n";
        throw;
    }
    return tasks;
}
TaskSet ReadTaskSet(string path, string priorityType = "RM")
{
    // some default parameters in this function
    string delimiter = ",";
    string token;
    string line;
    size_t pos = 0;

    vector<Task> taskSet;

    fstream file;
    file.open(path, ios::in);
    if (file.is_open())
    {
        string line;
        while (getline(file, line))
        {
            if (!(line[0] >= '0' && line[0] <= '9'))
                continue;
            vector<int> dataInLine;
            while ((pos = line.find(delimiter)) != string::npos)
            {
                token = line.substr(0, pos);
                int temp = atoi(token.c_str());
                dataInLine.push_back(temp);
                line.erase(0, pos + delimiter.length());
            }
            dataInLine.push_back(atoi(line.c_str()));
            dataInLine.erase(dataInLine.begin());
            taskSet.push_back(Task(dataInLine));
        }
        TaskSet ttt(taskSet);
        ttt = Reorder(ttt, priorityType);
        return ttt;
    }
    else
    {
        cout << "The path does not exist in ReadTaskSet!\n";
        throw;
    }
}

bool CheckSchedulability(const TaskSet &taskSet)
{
    int N = taskSet.size();
    // for (int i = 0; i < N; i++)
    // {
    //     vector<Task> const_iterator first =
    //     TaskSet hpTasks(first, last);
    //     if (ResponseTimeAnalysis(task, hpTasks) > task.deadline)
    //         return false;
    // }
    return true;
}