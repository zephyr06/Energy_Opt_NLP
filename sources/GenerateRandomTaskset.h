#include <time.h>
#include <cmath>

#include <boost/program_options/options_description.hpp>

#include "Tasks.h"
#include "Parameters.h"

namespace po = boost::program_options;

vector<double> Uunifast(int N, double utilAll)
{
    double sumU = utilAll;
    vector<double> utilVec(N, 0);

    double nextU;
    for (size_t i = 1; i < N; i++)
    {

        nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
        utilVec[i - 1] = sumU - nextU;
        sumU = nextU;
    }
    utilVec[N - 1] = nextU;
    return utilVec;
}

TaskSet GenerateTaskSet(int N, double totalUtilization,
                        int numberOfProcessor, int periodMin,
                        int periodMax)
{
    vector<double> utilVec = Uunifast(N, totalUtilization);
    TaskSet tasks;
    int periodMaxRatio = periodMax / periodMin;

    for (int i = 0; i < N; i++)
    {

        int periodCurr = (1 + rand() % periodMaxRatio) * periodMin;
        double randomRatio = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        Task task(0, periodCurr,
                  ceil(periodCurr * utilVec[i]) * 0.1 * randomRatio,
                  ceil(periodCurr * utilVec[i]),
                  periodCurr, i,
                  rand() % numberOfProcessor);
        tasks.push_back(task);
    }
    return tasks;
}
void WriteTaskSets(ofstream &file, TaskSet &tasks)
{
    int N = tasks.size();
    file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId\n";
    for (int i = 0; i < N; i++)
    {
        file << tasks[i].id << "," << tasks[i].offset << ","
             << tasks[i].period << "," << tasks[i].overhead << ","
             << tasks[i].executionTime << "," << tasks[i].deadline
             << "," << tasks[i].processorId << "\n";
    }
}
