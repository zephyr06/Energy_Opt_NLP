#pragma once
// this file records all the global variables
#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <vector>
namespace rt_num_opt {

cv::FileStorage ConfigParameters(
    "/home/zephyr/Programming/Energy_Opt_NLP/sources/parameters.yaml",
    cv::FileStorage::READ);

// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
// int TASK_NUMBER_DYNAMIC = 10;
int TASK_NUMBER = 0;
int count1 = 0;
int count2 = 0;
double weightEnergy = (double)ConfigParameters["weightEnergy"];
double punishmentInBarrier =
    weightEnergy * (double)ConfigParameters["punishmentInBarrier"];
double eliminateTol = (double)ConfigParameters["eliminateTol"];
double weightDrawBegin = (double)ConfigParameters["weightDrawBegin"];
double weightDrawEnd = (double)ConfigParameters["weightDrawEnd"];
double MaxComputationTimeRestrict =
    (double)ConfigParameters["MaxComputationTimeRestrict"];
double eliminateJacobianThreshold =
    (double)ConfigParameters["eliminateJacobianThreshold"];
double weightSchedulability = (double)ConfigParameters["weightSchedulability"];
double weightHardConstraint = (double)ConfigParameters["weightHardConstraint"];
double weightSchedulabilityMax =
    (double)ConfigParameters["weightSchedulabilityMax"];
double weightSchedulabilityMin =
    (double)ConfigParameters["weightSchedulabilityMin"];
double weightSchedulabilityStep =
    (double)ConfigParameters["weightSchedulabilityStep"];

double gradientModify = (double)ConfigParameters["gradientModify"];

int maxIterationsOptimizer = (int)ConfigParameters["maxIterationsOptimizer"];
int whether_IPM = (int)ConfigParameters["whether_IPM"];

int LLCompareWithGeneralizedElimination =
    (int)ConfigParameters["LLCompareWithGeneralizedElimination"];
int printFailureFile = (int)ConfigParameters["printFailureFile"];
int EnergyMode = (int)ConfigParameters["EnergyMode"];
int elimIte = (int)ConfigParameters["elimIte"];
int executionTimeModel = (int)ConfigParameters["executionTimeModel"];
int enableIPM = (int)ConfigParameters["enableIPM"];
const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
int enableMaxComputationTimeRestrict =
    (int)ConfigParameters["enableMaxComputationTimeRestrict"];
int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
const double utilTol = (double)ConfigParameters["utilTol"];
double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg =
    (double)ConfigParameters["deltaInitialDogleg"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int weightEnergyMaxOrder = (int)ConfigParameters["weightEnergyMaxOrder"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
double relativeErrorTolerance =
    (double)ConfigParameters["relativeErrorTolerance"];
double relativeErrorToleranceMin =
    (double)ConfigParameters["relativeErrorToleranceMin"];
const double relativeErrorToleranceInit =
    (double)ConfigParameters["relativeErrorToleranceInit"];
// enableReorder: 0 means no re-order, 1 means purely based on period with
// random priority assignment for tasks with same period, 2 means mainly based
// on period, prefer tasks with smaller execution time if tasks have same
// period;
int enableReorder = (int)ConfigParameters["enableReorder"];
int MaxLoopControl = (int)ConfigParameters["MaxLoopControl"];

double disturb_init = (double)ConfigParameters["disturb_init"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
int optimizerType = (int)ConfigParameters["optimizerType"];
double disturb_step = (double)ConfigParameters["disturb_step"];
double relativeErrorToleranceOuterLoop =
    (double)ConfigParameters["relativeErrorToleranceOuterLoop"];
double disturb_max = (double)ConfigParameters["disturb_max"];

const double punishmentFrequency =
    (double)ConfigParameters["punishmentFrequency"];
const std::string testDataSetName =
    (std::string)ConfigParameters["testDataSetName"];
std::string roundTypeInClamp =
    (std::string)ConfigParameters["roundTypeInClamp"];
std::string verbosityLM = (std::string)ConfigParameters["verbosityLM"];
std::string linearOptimizerType =
    (std::string)ConfigParameters["linearOptimizerType"];

std::string clampTypeMiddle = (std::string)ConfigParameters["clampTypeMiddle"];
std::string controlPath = (std::string)ConfigParameters["controlPath"];
std::string runMode = (std::string)ConfigParameters["runMode"];
std::string batchOptimizeFolder =
    (std::string)ConfigParameters["batchOptimizeFolder"];
const double parallelFactor = (double)ConfigParameters["parallelFactor"];
const std::string readTaskMode = (std::string)ConfigParameters["readTaskMode"];
const int debugMode = (int)ConfigParameters["debugMode"];
const int adjustEliminateMaxIte =
    (int)ConfigParameters["adjustEliminateMaxIte"];
int core_m_dag = (int)ConfigParameters["core_m_dag"];
int baselineLLCompare =
    (int)ConfigParameters["baselineLLCompare"];  // baselineLLCompare: 1 means
                                                 // Zhao20, 2 means MILP

double granularity_FindUnsustainable =
    (double)ConfigParameters["granularity_FindUnsustainable"];
const int taskSetSize_FindUnsustainable =
    (int)ConfigParameters["taskSetSize_FindUnsustainable"];

int maxNode_GenerateTaskSet = (int)ConfigParameters["maxNode_GenerateTaskSet"];

int printRTA = (int)ConfigParameters["printRTA"];
const double relErrorTolIPM = (double)ConfigParameters["relErrorTolIPM"];
const double eliminateStep = (double)ConfigParameters["eliminateStep"];
double frequencyRatio = (double)ConfigParameters["frequencyRatio"];
double timeScaleFactor = (double)ConfigParameters["timeScaleFactor"];
double SkipRateFindElimination =
    (double)ConfigParameters["SkipRateFindElimination"];
double jacobianScale = (double)ConfigParameters["jacobianScale"];

int whether_ls = (int)ConfigParameters["whether_ls"];

int setDiagonalDamping = (int)ConfigParameters["setDiagonalDamping"];
int whetherWriteNasriTaskSet =
    (int)ConfigParameters["whetherWriteNasriTaskSet"];
std::vector<int> PeriodSetAM;
void ReadVec(std::string str) {
    YAML::Node config = YAML::LoadFile(
        "/home/zephyr/Programming/Energy_Opt_NLP/sources/parameters.yaml");
    YAML::Node nodeVecPeriodSet = config[str];
    for (size_t i = 0; i < nodeVecPeriodSet.size(); i++) {
        PeriodSetAM.push_back(nodeVecPeriodSet[i].as<int>());
    }
}

}  // namespace rt_num_opt