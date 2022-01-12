#pragma once
#include "DAG_Task.h"

class RTA_DAG : public RTA_BASE<DAG_Model>
{
public:
    static double RTA_Common_Warm(double beginTime, const DAG_Model &tasks, int index)
    {
        return 0;
    }
    static double RTA_Common(const DAG_Model &tasks, int index)
    {
        return RTA_Common_Warm(0, tasks, index);
    }
};