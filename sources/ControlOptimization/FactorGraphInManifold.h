
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
// #include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
namespace rt_num_opt {

template <class TaskSetType, class Schedul_Analysis>
struct FactorGraphInManifold {
    // TODO: pass by reference
    static VectorDynamic ExtractResults(const gtsam::Values &result,
                                        const TaskSet &tasks) {
        VectorDynamic periods = GetParameterVD<double>(tasks, "period");
        for (uint i = 0; i < tasks.size(); i++) {
            if (result.exists(GenerateKey(i, "period"))) {
                periods(i, 0) =
                    result.at<VectorDynamic>(GenerateKey(i, "period"))(0, 0);
            }
        }
        return periods;
    }

    inline static Schedul_Analysis GenerateSchedul_Analysis(
        const TaskSetType &taskSetTypeRef, const gtsam::Values &x,
        const TaskSet &tasks) {
        TaskSetType tasksCurr = taskSetTypeRef;
        tasksCurr.UpdateTaskSet(tasks);
        UpdateTaskSetPeriod(tasksCurr.tasks_,
                            ExtractResults(x, tasksCurr.tasks_));
        return Schedul_Analysis(tasksCurr);
    }

    inline static Schedul_Analysis GenerateSchedul_Analysis(
        TaskSetType taskSetTypeRef, const TaskSet &tasks) {
        taskSetTypeRef.UpdateTaskSet(tasks);
        return Schedul_Analysis(taskSetTypeRef);
    }

    class ControlObjFactor : public gtsam::NoiseModelFactor {
       public:
        // data members
        TaskSet tasks;
        int index;
        VectorDynamic coeff;
        VectorDynamic rtaBase;
        int dimension;
        std::vector<gtsam::Symbol> keyVec;
        LambdaMultiKey f_with_RTA;
        LambdaMultiKey f_without_RTA;
        TaskSetType taskSetType;

        ControlObjFactor(std::vector<gtsam::Symbol> &keyVec,
                         const TaskSet &tasks, int index,
                         const VectorDynamic &coeff,
                         const VectorDynamic &rtaBase,
                         gtsam::SharedNoiseModel model,
                         const TaskSetType &taskSetType)
            : gtsam::NoiseModelFactor(model, keyVec),
              tasks(tasks),
              index(index),
              coeff(coeff),
              rtaBase(rtaBase),
              dimension(keyVec.size()),
              keyVec(keyVec),
              taskSetType(taskSetType) {
            double c = coeff[2 * index];
            f_without_RTA = [index, c](const gtsam::Values &x) {
                BeginTimer("f_without_RTA");
                VectorDynamic error = GenerateVectorDynamic(2);
                if (!x.exists(GenerateKey(index, "period"))) {
                    return error;
                }

                error(0) =
                    c * x.at<VectorDynamic>(GenerateKey(index, "period"))(0, 0);
                if (!whether_ls) {
                    error(0) = pow(error(0), 0.5);
                }
                error(1) = 0;
                EndTimer("f_without_RTA");
                return error;
            };

            f_with_RTA = [tasks, index, coeff, taskSetType,
                          rtaBase](const gtsam::Values &x) {
                BeginTimer("f_with_RTA");
                VectorDynamic error = GenerateVectorDynamic(2);
                // TaskSet tasksCurr = tasks;
                // UpdateTaskSetPeriod(tasksCurr, ExtractResults(x, tasks));
                // Schedul_Analysis r(tasksCurr);
                // Schedul_Analysis r =
                //     GenerateSchedul_Analysis(taskSetType, x, tasks);
                TaskSetType taskSetTypeCurr = taskSetType;
                TaskSet tasks_with_optimized_period = tasks;
                UpdateTaskSetPeriod(
                    tasks_with_optimized_period,
                    ExtractResults(x, tasks_with_optimized_period));
                taskSetTypeCurr.UpdateTaskSet(tasks_with_optimized_period);
                Schedul_Analysis r(taskSetTypeCurr);
                double rta = r.RTA_Common_Warm(rtaBase(index), index);
                if (!x.exists(GenerateKey(index, "period"))) {
                    error(0) = rta * coeff[2 * index + 1];
                } else {
                    error(0) =
                        rta * coeff[2 * index + 1] +
                        coeff[2 * index] * x.at<VectorDynamic>(GenerateKey(
                                               index, "period"))(0, 0);
                }

                if (!whether_ls) {
                    error(0) = pow(error(0), 0.5);
                }

                error(1) =
                    HingeLoss(taskSetTypeCurr.tasks_[index].period - rta);
                EndTimer("f_with_RTA");
                // std::cout << std::setprecision(12) <<
                // x.at<VectorDynamic>(GenerateKey(index, "period"))(0, 0) << ",
                // " << error(0)
                //           << std::setprecision(5) << std::endl;
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
                for (int i = 0; i < dimension; i++) {
                    if (exactJacobian) {
                        NormalErrorFunction1D f =
                            [x, i, this](const VectorDynamic xi) {
                                gtsam::Symbol a = keyVec.at(i);
                                gtsam::Values xx = x;
                                xx.update(a, xi);
                                return f_with_RTA(xx);
                            };
                        (*H)[i] = NumericalDerivativeDynamic(
                            f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                    } else {
                        VectorDynamic jacob = GenerateVectorDynamic(2);
                        int taskId = AnalyzeKey(keyVec[i]);
                        if (taskId == index) {
                            jacob(0) = 0.5 * coeff(2 * index) / result(0);
                            if (optimizerType !=
                                2)  // avoid numerically indetermined system
                            {
                                if (jacob(0) / result(0) < 1e-5) {
                                    jacob(0) = result(0) * 1e-5;
                                }
                            }
                        }
                        (*H)[i] = jacob;
                    }

                    (*H)[i] = (*H)[i] * jacobianScale;
                }
                if (debugMode == 1) {
                    std::lock_guard<std::mutex> lock(mtx);
                    std::cout << Color::blue;
                    // PrintControlValues(x);
                    // x.print();
                    std::cout << Color::def;
                }
            }
            EndTimer("RTARelatedFactor_unwhitenedError");
            return result;
        }
    };

    static ControlObjFactor GenerateControlObjFactor(
        std::vector<bool> maskForElimination, const TaskSet &tasks, int index,
        VectorDynamic &coeff, VectorDynamic &rtaBase,
        const TaskSetType &taskSetType) {
        BeginTimer(__func__);
        std::vector<gtsam::Symbol> keys;
        keys.reserve(index);
        for (int i = 0; i <= index; i++) {
            if (!maskForElimination[i]) {
                keys.push_back(GenerateKey(i, "period"));
            }
        }

        VectorDynamic sigma = GenerateVectorDynamic(2);
        sigma << noiseModelSigma, noiseModelSigma / weightSchedulability;
        auto model = gtsam::noiseModel::Diagonal::Sigmas(sigma);
        EndTimer(__func__);
        return ControlObjFactor(keys, tasks, index, coeff, rtaBase, model,
                                taskSetType);
    }

    /* whether task 'index' has free dependent variables*/
    static bool HasDependency(int index,
                              std::vector<bool> &maskForElimination) {
        for (int i = 0; i <= index; i++) {
            if (!maskForElimination[i])
                return true;
        }
        return false;
    }

    static gtsam::NonlinearFactorGraph BuildControlGraph(
        std::vector<bool> maskForElimination, const TaskSet &tasks,
        VectorDynamic &coeff, const TaskSetType &taskSetType) {
        BeginTimer(__func__);
        gtsam::NonlinearFactorGraph graph;
        double periodMax =
            GetParameterVD<double>(tasks, "executionTime").sum() * 5;
        auto modelNormal =
            gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(
            1, noiseModelSigma / weightHardConstraint);
        // NOTICE: the usage here is valid because only RTA_LL utilizes a valid
        // warm_start, i.e., rtaBAse
        VectorDynamic rtaBase = RTAVector(tasks);
        for (uint i = 0; i < tasks.size(); i++) {
            if (!maskForElimination[i]) {
                graph.emplace_shared<LargerThanFactor1D>(
                    GenerateKey(i, "period"), tasks.at(i).executionTime,
                    modelPunishmentSoft1);
                graph.emplace_shared<SmallerThanFactor1D>(
                    GenerateKey(i, "period"), periodMax, modelPunishmentSoft1);
            }

            // clean the following
            if (HasDependency(i, maskForElimination)) {
                auto factor = GenerateControlObjFactor(
                    maskForElimination, tasks, i, coeff, rtaBase, taskSetType);
                graph.add(factor);
            }
        }
        EndTimer(__func__);
        return graph;
    }

    static gtsam::Values GenerateInitialFG(
        const TaskSet &tasks, std::vector<bool> &maskForElimination) {
        gtsam::Values initialEstimateFG;
        for (uint i = 0; i < tasks.size(); i++) {
            if (!maskForElimination[i]) {
                initialEstimateFG.insert(
                    GenerateKey(i, "period"),
                    GenerateVectorDynamic1D(tasks.at(i).period));
            }
        }
        return initialEstimateFG;
    }

    static void FindEliminatedVariables(TaskSet &tasks,
                                        std::vector<bool> &maskForElimination,
                                        const TaskSetType &taskSetType,
                                        double disturb = disturb_init) {
        BeginTimer(__func__);
        Schedul_Analysis r = GenerateSchedul_Analysis(taskSetType, tasks);
        VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
        bool whether_new_eliminate = false;
        while (!whether_new_eliminate && disturb <= disturb_max) {
            for (uint i = 0; i < tasks.size(); i++) {
                tasks[i].period -= disturb;

                Schedul_Analysis r1 =
                    GenerateSchedul_Analysis(taskSetType, tasks);
                // RTA_LL r1(tasks);
                VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet(rtaBase);
                if ((rtaBase - rtaCurr).array().abs().maxCoeff() >= disturb ||
                    !r1.CheckSchedulabilityDirect(rtaCurr))

                {
                    if (!maskForElimination[i])
                        whether_new_eliminate = true;
                    maskForElimination[i] = true;
                }
                tasks[i].period += disturb;
            }
            if (!whether_new_eliminate)
                disturb *= disturb_step;
            if (debugMode == 1) {
                std::lock_guard<std::mutex> lock(mtx);
                for (auto a : maskForElimination) std::cout << a << ", ";
                std::cout << std::endl;
            }
        }
        // std::cout << "eliminate tol: " << disturb << std::endl;
        EndTimer(__func__);
    }

    static double RealObj(const TaskSet &tasks, const VectorDynamic &coeff,
                          const TaskSetType &taskSetType) {
        BeginTimer(__func__);
        double res = 0;
        Schedul_Analysis r = GenerateSchedul_Analysis(taskSetType, tasks);
        // RTA_LL r(tasks);
        VectorDynamic rta = r.ResponseTimeOfTaskSet();
        for (uint i = 0; i < tasks.size(); i++) {
            res += coeff.coeffRef(i * 2, 0) * tasks[i].period;
            res += coeff.coeffRef(i * 2 + 1, 0) * rta(i, 0);
        }
        EndTimer(__func__);
        return res;
    }
};
}  // namespace rt_num_opt