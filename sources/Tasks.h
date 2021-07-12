#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
// #include <filesystem>
#include <boost/integer/common_factor.hpp>

#include <CppUnitLite/TestHarness.h>

#include "colormod.h"

#include "Parameters.h"
using namespace std;

#define TaskSet vector<Task>
Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier blue(Color::FG_BLUE);
Color::Modifier def(Color::FG_DEFAULT);

class Task
{
public:
    // Member list
    int offset;
    int period;
    int overhead;
    double executionTime;
    int deadline;

    // initializer

    Task() : offset(0), period(0),
             overhead(0), executionTime(0.0),
             deadline(0) {}
    Task(int offset, int period, int overhead, double executionTime,
         int deadline) : offset(offset), period(period),
                         overhead(overhead), executionTime(executionTime),
                         deadline(deadline) {}

    /**
 * only used in ReadTaskSet because the input parameter's type is int
 **/
    Task(vector<int> dataInLine)
    {
        if (dataInLine.size() != 5)
        {
            cout << red << "The length of dataInLine in Task constructor is wrong! Must be 5!\n"
                 << def;
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

    double utilization()
    {
        return double(executionTime) / period;
    }

    int slack()
    {
        return deadline - executionTime;
    }
};

void Print(TaskSet &tasks)
{
    cout << "The task set is printed as follows" << endl;
    for (auto &task : tasks)
        task.print();
}

template <typename T>
vector<T> GetParameter(const TaskSet &taskset, string parameterType)
{
    uint N = taskset.size();
    vector<T> parameterList;
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
            cout << red << "parameterType in GetParameter is not recognized!\n"
                 << def;
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
bool compareUtilization(Task task1, Task task2)
{
    return task1.utilization() < task2.utilization();
};

double Utilization(const TaskSet &tasks)
{
    vector<int> periodHigh = GetParameter<int>(tasks, "period");
    vector<double> executionTimeHigh = GetParameter<double>(tasks, "executionTime");
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
        cout << red << "Empty task set in HyperPeriod()!\n";
        throw;
    }
    else
    {
        long long int hyper = tasks[0].period;
        for (int i = 1; i < N; i++)
        {
            hyper = lcm(hyper, tasks[i].period);
            if (hyper < 0 || hyper > LLONG_MAX)
            {
                cout << red << "The hyper-period over flows!" << def << endl;
                throw;
            }
        }
        return hyper;
    }
}

TaskSet Reorder(TaskSet tasks, string priorityType)
{
    if (priorityType == "RM")
    {
        sort(tasks.begin(), tasks.end(), comparePeriod);
    }
    else if (priorityType == "utilization")
    {
        sort(tasks.begin(), tasks.end(), compareUtilization);
    }
    else if (priorityType == "orig")
    {
        ;
    }
    else
    {
        cout << red << "Unrecognized priorityType in Reorder!\n"
             << def;
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
        if (debugMode == 1)
            cout << "Finish reading the data file succesfully!\n";
        return ttt;
    }
    else
    {
        cout << red << "The path does not exist in ReadTaskSet!\n"
             << def;
        throw;
    }
}
