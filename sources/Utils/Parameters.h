#pragma once
// this file records all the global variables
#include <yaml-cpp/yaml.h>

// #include <opencv2/core/core.hpp>

#include <vector>
namespace rt_num_opt {

// cv::FileStorage ConfigParameters(
//     "/home/zephyr/Programming/Energy_Opt_NLP/sources/parameters.yaml",
//     cv::FileStorage::READ);
const std::string PROJECT_PATH = std::string(PROJECT_ROOT_DIR) + "/";
YAML::Node loaded_doc =
    YAML::LoadFile(PROJECT_PATH + "sources/parameters.yaml");

double MaxLoopControl = loaded_doc["MaxLoopControl"].as<double>();
int enableReorder = loaded_doc["enableReorder"].as<int>();
int TASK_NUMBER = 0;
double weightEnergy = loaded_doc["weightEnergy"].as<double>();

double punishmentInBarrier = loaded_doc["punishmentInBarrier"].as<double>();
double eliminateTol = loaded_doc["eliminateTol"].as<double>();
double MaxComputationTimeRestrict =
    loaded_doc["MaxComputationTimeRestrict"].as<double>();
double weightSchedulability = loaded_doc["weightSchedulability"].as<double>();
double weightHardConstraint = loaded_doc["weightHardConstraint"].as<double>();
double gradientModify = loaded_doc["gradientModify"].as<double>();
double coolingRateSA = loaded_doc["coolingRateSA"].as<double>();
double utilTol = loaded_doc["utilTol"].as<double>();

double maxIterationsOptimizer =
    loaded_doc["maxIterationsOptimizer"].as<double>();
int whether_IPM = loaded_doc["whether_IPM"].as<int>();

int LLCompareWithGeneralizedElimination =
    loaded_doc["LLCompareWithGeneralizedElimination"].as<int>();
int printFailureFile = loaded_doc["printFailureFile"].as<int>();
int EnergyMode = loaded_doc["EnergyMode"].as<int>();
int elimIte = loaded_doc["elimIte"].as<int>();
int executionTimeModel = loaded_doc["executionTimeModel"].as<int>();
int enableMaxComputationTimeRestrict =
    loaded_doc["enableMaxComputationTimeRestrict"].as<int>();
int exactJacobian = loaded_doc["exactJacobian"].as<int>();
const int temperatureSA = loaded_doc["temperatureSA"].as<int>();
double deltaOptimizer = loaded_doc["deltaOptimizer"].as<double>();
const double initialLambda = loaded_doc["initialLambda"].as<double>();
const double lowerLambda = loaded_doc["lowerLambda"].as<double>();
const double upperLambda = loaded_doc["upperLambda"].as<double>();
double noiseModelSigma = loaded_doc["noiseModelSigma"].as<double>();
const double deltaInitialDogleg = loaded_doc["deltaInitialDogleg"].as<double>();

const int randomInitialize = loaded_doc["randomInitialize"].as<int>();
const int SA_iteration = loaded_doc["SA_iteration"].as<int>();
double relativeErrorTolerance =
    loaded_doc["relativeErrorTolerance"].as<double>();
double relativeErrorToleranceMin =
    loaded_doc["relativeErrorToleranceMin"].as<double>();
const double relativeErrorToleranceInit =
    loaded_doc["relativeErrorToleranceInit"].as<double>();

double Priority_assignment_adjustment_threshold =
    loaded_doc["Priority_assignment_adjustment_threshold"].as<double>();
double disturb_init = loaded_doc["disturb_init"].as<double>();
int optimizerType = loaded_doc["optimizerType"].as<int>();
double disturb_step = loaded_doc["disturb_step"].as<double>();
double relativeErrorToleranceOuterLoop =
    loaded_doc["relativeErrorToleranceOuterLoop"].as<double>();
double disturb_max = loaded_doc["disturb_max"].as<double>();

const double punishmentFrequency =
    loaded_doc["punishmentFrequency"].as<double>();
const std::string testDataSetName =
    loaded_doc["testDataSetName"].as<std::string>();
std::string roundTypeInClamp = loaded_doc["roundTypeInClamp"].as<std::string>();
std::string verbosityLM = loaded_doc["verbosityLM"].as<std::string>();
std::string linearOptimizerType =
    loaded_doc["linearOptimizerType"].as<std::string>();

std::string clampTypeMiddle = loaded_doc["clampTypeMiddle"].as<std::string>();
std::string controlPath = loaded_doc["controlPath"].as<std::string>();
std::string runMode = loaded_doc["runMode"].as<std::string>();
std::string batchOptimizeFolder =
    loaded_doc["batchOptimizeFolder"].as<std::string>();
const double parallelFactor = loaded_doc["parallelFactor"].as<double>();
const std::string readTaskMode = loaded_doc["readTaskMode"].as<std::string>();
const int debugMode = loaded_doc["debugMode"].as<int>();
const int adjustEliminateMaxIte = loaded_doc["adjustEliminateMaxIte"].as<int>();
int core_m_dag = loaded_doc["core_m_dag"].as<int>();
int baselineLLCompare =
    loaded_doc["baselineLLCompare"].as<int>();  // baselineLLCompare: 1 means
                                                // Zhao20, 2 means MILP

// *********************************************
int Period_Round_For_Control_Opt =
    loaded_doc["Period_Round_For_Control_Opt"].as<int>();
double granularity_FindUnsustainable =
    loaded_doc["granularity_FindUnsustainable"].as<double>();
const int taskSetSize_FindUnsustainable =
    loaded_doc["taskSetSize_FindUnsustainable"].as<int>();

double Job_Limit_Scheduling = loaded_doc["Job_Limit_Scheduling"].as<double>();
int maxNode_GenerateTaskSet = loaded_doc["maxNode_GenerateTaskSet"].as<int>();

int printRTA = loaded_doc["printRTA"].as<int>();
const double relErrorTolIPM = loaded_doc["relErrorTolIPM"].as<double>();
const double eliminateStep = loaded_doc["eliminateStep"].as<double>();
double frequencyRatio = 0;
double timeScaleFactor = loaded_doc["timeScaleFactor"].as<double>();
double PeriodRoundQuantum = loaded_doc["PeriodRoundQuantum"].as<double>();
double jacobianScale = loaded_doc["jacobianScale"].as<double>();
double control_sort_obj_coeff_weight =
    loaded_doc["control_sort_obj_coeff_weight"].as<double>();
double control_sort_exec_weight =
    loaded_doc["control_sort_exec_weight"].as<double>();
double weight_priority_assignment =
    loaded_doc["weight_priority_assignment"].as<double>();

double OverallTimeLimit = loaded_doc["OverallTimeLimit"].as<double>();
double Nasri19Param_timeout = loaded_doc["Nasri19Param_timeout"].as<double>();
double Nasri19Param_max_depth =
    loaded_doc["Nasri19Param_max_depth"].as<double>();

double Priority_assignment_threshold_incremental =
    loaded_doc["Priority_assignment_threshold_incremental"].as<double>();
double Obj_Pow = loaded_doc["Obj_Pow"].as<double>();
int whether_ls = loaded_doc["whether_ls"].as<int>();

int EvaluatePA_FailedThreshold =
    loaded_doc["EvaluatePA_FailedThreshold"].as<int>();
int setDiagonalDamping = loaded_doc["setDiagonalDamping"].as<int>();
int whetherWriteNasriTaskSet = 0;
}  // namespace rt_num_opt