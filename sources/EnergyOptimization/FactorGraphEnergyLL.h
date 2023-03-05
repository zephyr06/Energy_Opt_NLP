#pragma once
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <math.h>

#include <Eigen/Dense>
#include <chrono>

#include "sources/EnergyOptimization/Energy.h"
#include "sources/EnergyOptimization/EnergyOptimize.h"
#include "sources/EnergyOptimization/FrequencyModel.h"
#include "sources/MatrixConvenient.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/TaskModel/Tasks.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/GlobalVariables.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/utils.h"
namespace rt_num_opt {
struct FactorGraphEnergyLL {
    static double FindEliminatedVariables(TaskSet &tasks,
                                          std::vector<bool> &maskForElimination,
                                          bool &whether_new_eliminate,
                                          double disturb = disturb_init) {
        BeginTimer(__func__);
        whether_new_eliminate = false;
        if (debugMode == 1) {
            std::cout << GetParameterVD<double>(tasks, "executionTime")
                      << std::endl;
        }

        RTA_LL r(tasks);
        VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
        // bool whether_new_eliminate = false;
        while (!whether_new_eliminate && disturb <= disturb_max) {
            for (uint i = 0; i < tasks.size(); i++) {
                if (maskForElimination[i]) continue;
                tasks[i].executionTime += disturb;
                RTA_LL r1(tasks);
                // VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet(rtaBase);
                if (!r1.CheckSchedulability(1 == debugMode) ||
                    (enableMaxComputationTimeRestrict &&
                     tasks[i].executionTime > tasks[i].executionTimeOrg *
                                                  MaxComputationTimeRestrict))
                // TODO: more analytic way
                {
                    if (!maskForElimination[i]) whether_new_eliminate = true;
                    maskForElimination[i] = true;
                } else {
                    maskForElimination[i] = false;
                }
                tasks[i].executionTime -= disturb;
            }
            if (!whether_new_eliminate) disturb *= disturb_step;
            if (debugMode == 1) {
                std::lock_guard<std::mutex> lock(mtx);
                for (auto a : maskForElimination) std::cout << a << ", ";
                std::cout << std::endl;
            }
        }
        EndTimer(__func__);
        return disturb;
    }
};
}  // namespace rt_num_opt