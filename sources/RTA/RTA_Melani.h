#pragma once
#include "sources/TaskModel/DAG_Melani.h"
#include "sources/RTA/RTA_BASE.h"
namespace rt_num_opt
{
  class RTA_DAG : public RTA_BASE<TaskSetDAG>
  {

  public:
    std::vector<double> rta;

    RTA_DAG() {}
    RTA_DAG(const TaskSetDAG &tasksI)
    {
      tasks = tasksI;
      rta.reserve(tasks.N);
      for (int i = 0; i < tasks.N; i++)
        rta.push_back(-1);
    }

    double RTA_Common_Warm(double beginTime, int index) override
    {
      if (index < 0 || index > tasks.N)
        CoutError("Index out of bound in RTA_DAG, RTA_Common_Warm");
      if (rta[index] != -1)
      {
        return rta[index];
      }
      else if (index == 0)
      {
        rta[0] = BasicComputation(0);
        return rta[0];
      }
      else
      {
        double rtaLeft = 0;
        double rtaBasic = BasicComputation(index);
        double rtaRight = rtaBasic;
        double cycleCount = 0;
        while (rtaLeft < rtaRight)
        {
          rtaLeft = rtaRight;
          rtaRight = rtaBasic + Interference(index, rtaLeft) / core_m_dag;
          cycleCount++;
          if (cycleCount > 1000)
          {
            CoutWarning("cycleCount exceeds 1000!");
            rtaLeft = INT32_MAX;
            break;
          }
        }
        rta[index] = rtaLeft;
        if (rta[index] < 0)
          CoutError("Negative response time detected!");
        return rtaLeft;
      }
      return 0;
    }

    double RTA_Common(int index)
    {
      return RTA_Common_Warm(0, index);
    }

    inline double BasicComputation(int index)
    {
      return tasks.longestVec_[index] + 1.0 / core_m_dag * (tasks.volumeVec_[index] - tasks.longestVec_[index]);
    }

    double Interference(int index, double rtaIte)
    {
      double interf = 0;
      for (int i = 0; i < index; i++)
      {
        double carry = rtaIte + RTA_Common_Warm(0, i) -
                       tasks.volumeVec_[i] / core_m_dag;
        double firstItem = floor((carry) /
                                 tasks.tasks_[i].period);
        double secondItem = min(tasks.volumeVec_[i], core_m_dag * (int(std::ceil(carry)) % int(tasks.tasks_[i].period)));
        interf += firstItem + secondItem;
      }
      return interf;
    }
  };
} // namespace rt_num_opt