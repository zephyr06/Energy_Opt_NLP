
#pragma once
#include <CppUnitLite/TestHarness.h>

#include <chrono>
#include <numeric>
#include <string>
#include <utility>

// #include "sources/ControlOptimization/CoeffFactor.h"
// #include "sources/ControlOptimization/RTAFactor.h"
#include "sources/ControlOptimization/LinearEqualityFactor.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
namespace rt_num_opt {

VectorDynamic GetPeriodVecNasri19(const DAG_Nasri19 &tasks_dag) {
    VectorDynamic res = GenerateVectorDynamic(tasks_dag.SizeDag());
    for (uint i = 0; i < tasks_dag.SizeDag(); i++)
        res(i, 0) = tasks_dag.getDag(i).GetPeriod();
    return res;
}

template <class TaskSetType, class Schedul_Analysis>
struct FactorGraphNasri {
    // TODO: pass by reference
    static VectorDynamic ExtractNodePeriodVec(
        const gtsam::Values &result, const TaskSetType &taskSetTypeRef) {
        VectorDynamic periods =
            GetParameterVD<double>(taskSetTypeRef, "period");
        size_t node_overall_count = 0;
        for (size_t taskId = 0; taskId < taskSetTypeRef.tasksVecNasri_.size();
             taskId++) {
            for (size_t nodeId = 0;
                 nodeId < taskSetTypeRef.tasksVecNasri_[taskId].tasks_.size();
                 nodeId++) {
                if (result.exists(GenerateKey(taskId, "period"))) {
                    periods(node_overall_count, 0) = result.at<VectorDynamic>(
                        GenerateKey(taskId, "period"))(0, 0);
                }
                node_overall_count++;
            }
        }
        return periods;
    }
    static VectorDynamic ExtractDAGPeriodVec(
        const gtsam::Values &result, const TaskSetType &taskSetTypeRef) {
        VectorDynamic periods = GenerateVectorDynamic(taskSetTypeRef.SizeDag());
        for (size_t taskId = 0; taskId < taskSetTypeRef.tasksVecNasri_.size();
             taskId++) {
            if (result.exists(GenerateKey(taskId, "period"))) {
                periods(taskId, 0) = result.at<VectorDynamic>(
                    GenerateKey(taskId, "period"))(0, 0);
            } else
                periods(taskId, 0) =
                    taskSetTypeRef.tasksVecNasri_[taskId].tasks_[0].period;
        }
        return periods;
    }

    static void UpdateTaskSetPeriodFromValues(TaskSetType &taskSetType,
                                              const gtsam::Values &result) {
        for (uint i = 0; i < taskSetType.SizeDag(); i++) {
            if (result.exists(GenerateKey(i, "period"))) {
                taskSetType.UpdatePeriod(
                    i,
                    result.at<VectorDynamic>(GenerateKey(i, "period"))(0, 0));
            }
        }
    }

    static void UpdateTaskSetWithPeriodVariable(
        TaskSetType &taskSetType, const VectorDynamic &periodVec) {
        for (uint i = 0; i < taskSetType.SizeDag(); i++) {
            taskSetType.UpdatePeriod(i, periodVec(i));
        }
    }

    static VectorDynamic GetControlObjVector(const VectorDynamic &rta,
                                             const VectorDynamic &periodVec,
                                             const TaskSetType &taskSetType,
                                             const VectorDynamic &coeff) {
        VectorDynamic error = GenerateVectorDynamic(2 * taskSetType.SizeNode());
        size_t node_overall_count = 0;
        for (size_t taskId = 0; taskId < taskSetType.tasksVecNasri_.size();
             taskId++) {
            for (size_t nodeId = 0;
                 nodeId < taskSetType.tasksVecNasri_[taskId].tasks_.size();
                 nodeId++) {
                // TODO: !! consider whether using periodVec or not!
                // double period_curr =
                //     taskSetType.tasksVecNasri_[taskId].tasks_[nodeId].period;
                double period_curr = periodVec(taskId, 0);
                error(2 * node_overall_count) =
                    period_curr * coeff[2 * node_overall_count] +
                    rta(node_overall_count) * coeff[2 * node_overall_count + 1];

                // least-square
                error(2 * node_overall_count) =
                    pow(error(2 * node_overall_count), 0.5);

                error(2 * node_overall_count + 1) =
                    HingeLoss(period_curr - rta(node_overall_count));
                node_overall_count++;
            }
        }
        return error;
    }

    static gtsam::NonlinearFactorGraph BuildControlGraph(
        const TaskSetType &taskSetType, std::vector<bool> maskForElimination,
        VectorDynamic &coeff) {
        BeginTimer(__func__);
        gtsam::NonlinearFactorGraph graph;
        auto modelNormal =
            gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(
            1, noiseModelSigma / weightHardConstraint);

        for (uint i = 0; i < taskSetType.SizeDag(); i++) {
            if (!maskForElimination[i]) {
                graph.emplace_shared<LargerThanFactor1D>(
                    GenerateKey(i, "period"), taskSetType.getDag(i).Volume(),
                    modelPunishmentSoft1);
                // graph.emplace_shared<SmallerThanFactor1D>(
                //     GenerateKey(i, "period"), periodMax,
                //     modelPunishmentSoft1);
            }
        }

        auto factor =
            GenerateControlObjFactor(maskForElimination, coeff, taskSetType);
        graph.add(factor);

        EndTimer(__func__);
        return graph;
    }

    static gtsam::Values GenerateInitialFG(
        const TaskSetType &taskSetType, std::vector<bool> &maskForElimination) {
        gtsam::Values initialEstimateFG;
        for (uint i = 0; i < taskSetType.SizeDag(); i++) {
            if (!maskForElimination[i]) {
                initialEstimateFG.insert(
                    GenerateKey(i, "period"),
                    GenerateVectorDynamic1D(taskSetType.getDag(i).GetPeriod()));
            }
        }
        return initialEstimateFG;
    }

    class ControlObjFactor : public gtsam::NoiseModelFactor {
       public:
        // data members
        VectorDynamic coeff;
        std::vector<gtsam::Symbol> keyVec;
        LambdaMultiKey f_with_RTA;
        TaskSetType taskSetType;
        std::vector<bool> maskForElimination;

        ControlObjFactor(std::vector<gtsam::Symbol> &keyVec,
                         const VectorDynamic &coeff,
                         gtsam::SharedNoiseModel model,
                         const TaskSetType &taskSetType,
                         const std::vector<bool> &maskForElimination)
            : gtsam::NoiseModelFactor(model, keyVec),
              coeff(coeff),
              keyVec(keyVec),
              taskSetType(taskSetType) {
            f_with_RTA = [coeff, maskForElimination,
                          taskSetType](const gtsam::Values &x) {
                BeginTimer("f_with_RTA");
                TaskSetType taskSetTypeRounded = taskSetType;
                UpdateTaskSetPeriodFromValues(taskSetTypeRounded, x);
                Schedul_Analysis r(taskSetTypeRounded);
                VectorDynamic rta = r.ResponseTimeOfTaskSet();
                VectorDynamic periodVec = ExtractDAGPeriodVec(x, taskSetType);
                VectorDynamic error = GetControlObjVector(
                    rta, periodVec, taskSetTypeRounded, coeff);
                EndTimer("f_with_RTA");
                return error;
            };
        }

        gtsam::Vector unwhitenedError(
            const gtsam::Values &x,
            boost::optional<std::vector<gtsam::Matrix> &> H =
                boost::none) const override {
            BeginTimer("RTARelatedFactor_unwhitenedError");
            gtsam::Vector result = f_with_RTA(x);
            if (H) {
                if (exactJacobian) {
                    for (uint i = 0; i < keyVec.size(); i++) {
                        NormalErrorFunction1D f =
                            [x, i, this](const VectorDynamic xi) {
                                gtsam::Symbol a = keyVec.at(i);
                                gtsam::Values xx = x;
                                xx.update(a, xi);
                                return f_with_RTA(xx);
                            };
                        (*H)[i] = NumericalDerivativeDynamic(
                            f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                        if (debugMode == 1) {
                            std::cout << (*H)[i] << "\n\n";
                        }
                    }
                } else {
                    TaskSetType taskSetTypeRounded = taskSetType;
                    UpdateTaskSetPeriodFromValues(taskSetTypeRounded, x);
                    Schedul_Analysis r(taskSetTypeRounded);
                    VectorDynamic rta_base = r.ResponseTimeOfTaskSet();

                    for (uint i = 0; i < keyVec.size(); i++) {
                        NormalErrorFunction1D f =
                            [x, i, rta_base, this](const VectorDynamic xi) {
                                gtsam::Symbol a = keyVec.at(i);
                                gtsam::Values xx = x;
                                xx.update(a, xi);
                                VectorDynamic periodVec =
                                    ExtractDAGPeriodVec(xx, taskSetType);
                                return GetControlObjVector(rta_base, periodVec,
                                                           taskSetType, coeff);
                            };
                        (*H)[i] = NumericalDerivativeDynamic(
                            f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                        if (debugMode == 1) {
                            std::cout << (*H)[i] << "\n\n";
                        }
                    }
                }

                for (uint i = 0; i < keyVec.size(); i++)
                    (*H)[i] = (*H)[i] * jacobianScale;
                if (debugMode == 1) {
                    std::lock_guard<std::mutex> lock(mtx);
                    std::cout << Color::blue;

                    // for (uint i = 0; i < keyVec.size(); i++)
                    //     std::cout << (*H)[i] << "\n";
                    std::cout << "Error vector: " << result << "\n";
                    std::cout << Color::def;
                }
            }
            EndTimer("RTARelatedFactor_unwhitenedError");
            return result;
        }
    };

    static ControlObjFactor GenerateControlObjFactor(
        const std::vector<bool> &maskForElimination, VectorDynamic &coeff,
        const TaskSetType &taskSetType) {
        BeginTimer(__func__);
        std::vector<gtsam::Symbol> keys;
        keys.reserve(taskSetType.SizeNode());
        for (uint i = 0; i < taskSetType.SizeDag(); i++) {
            if (!maskForElimination.at(i)) {
                keys.push_back(GenerateKey(i, "period"));
            }
        }
        VectorDynamic sigma = GenerateVectorDynamic(2 * taskSetType.SizeNode());
        for (uint i = 0; i < sigma.rows() / 2; i++) {
            sigma(2 * i) = noiseModelSigma;
            sigma(2 * i + 1) = noiseModelSigma / weightSchedulability;
        }
        auto model = gtsam::noiseModel::Diagonal::Sigmas(sigma);
        EndTimer(__func__);
        return ControlObjFactor(keys, coeff, model, taskSetType,
                                maskForElimination);
    }

    static void FindEliminatedVariables(const TaskSetType &taskSetType,
                                        std::vector<bool> &maskForElimination,
                                        double disturb = disturb_init) {
        BeginTimer(__func__);
        Schedul_Analysis r(taskSetType);
        VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
        bool whether_new_eliminate = false;
        while (!whether_new_eliminate && disturb <= disturb_max) {
            for (uint i = 0; i < taskSetType.SizeDag(); i++) {
                // tasks[i].period -= disturb;
                TaskSetType taskSetTypeCurr = taskSetType;
                taskSetTypeCurr.AdjustPeriod(i, disturb * -1);
                Schedul_Analysis r1(taskSetTypeCurr);
                VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet();
                if ((rtaBase - rtaCurr).array().abs().maxCoeff() >= disturb ||
                    !r1.CheckSchedulabilityDirect(rtaCurr)) {
                    if (!maskForElimination[i])
                        whether_new_eliminate = true;
                    maskForElimination[i] = true;
                }
                // tasks[i].period += disturb;
                taskSetTypeCurr.AdjustPeriod(i, disturb * 1);
            }
            if (!whether_new_eliminate)
                disturb *= disturb_step;
            if (debugMode == 1) {
                std::lock_guard<std::mutex> lock(mtx);
                for (auto a : maskForElimination) std::cout << a << ", ";
                std::cout << std::endl;
            }
        }
        EndTimer(__func__);
    }

    static double RealObj(const TaskSetType &taskSetType,
                          const VectorDynamic &coeff) {
        BeginTimer(__func__);
        double res = 0;
        Schedul_Analysis r(taskSetType);
        VectorDynamic rta = r.ResponseTimeOfTaskSet();
        for (uint i = 0; i < taskSetType.tasks_.size(); i++) {
            res += coeff.coeffRef(i * 2, 0) * taskSetType.tasks_[i].period;
            res += coeff.coeffRef(i * 2 + 1, 0) * rta(i, 0);
        }
        EndTimer(__func__);
        return res;
    }
};
}  // namespace rt_num_opt