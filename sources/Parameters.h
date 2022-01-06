#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

cv::FileStorage ConfigParameters("/home/zephyr/Programming/Energy_Opt_NLP/sources/parameters.yaml", cv::FileStorage::READ);

using namespace std;
// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
// int TASK_NUMBER_DYNAMIC = 10;
int TASK_NUMBER = 0;
double weightEnergy = (double)ConfigParameters["weightEnergy"];
double punishmentInBarrier = weightEnergy * (double)ConfigParameters["punishmentInBarrier"];
double eliminateTol = (double)ConfigParameters["eliminateTol"];
double weightDrawBegin = (double)ConfigParameters["weightDrawBegin"];
double weightDrawEnd = (double)ConfigParameters["weightDrawEnd"];
double MaxComputationTimeRestrict = (double)ConfigParameters["MaxComputationTimeRestrict"];

int printFailureFile = (int)ConfigParameters["printFailureFile"];
int EnergyMode = (int)ConfigParameters["EnergyMode"];
int elimIte = (int)ConfigParameters["elimIte"];
int executionTimeModel = (int)ConfigParameters["executionTimeModel"];
int enableIPM = (int)ConfigParameters["enableIPM"];
const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
int enableMaxComputationTimeRestrict = (int)ConfigParameters["enableMaxComputationTimeRestrict"];
int exactJacobian = (int)ConfigParameters["exactJacobian"];
double computationBound = (double)ConfigParameters["computationBound"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
const double utilTol = (double)ConfigParameters["utilTol"];
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int weightEnergyMaxOrder = (int)ConfigParameters["weightEnergyMaxOrder"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
int optimizerType = (int)ConfigParameters["optimizerType"];
double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double punishmentFrequency = (double)ConfigParameters["punishmentFrequency"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
const string roundTypeInClamp = (string)ConfigParameters["roundTypeInClamp"];
string runMode = (string)ConfigParameters["runMode"];

const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const int granularityInBF = (int)ConfigParameters["granularityInBF"];
const double eliminateVariableThreshold = (double)ConfigParameters["eliminateVariableThreshold"];
const int debugMode = (int)ConfigParameters["debugMode"];

const double relErrorTolIPM = (double)ConfigParameters["relErrorTolIPM"];
const double weightStep = (double)ConfigParameters["weightStep"];
const double eliminateStep = (double)ConfigParameters["eliminateStep"];
const int iterationNumIPM_Max = (int)ConfigParameters["iterationNumIPM_Max"];
const double xTolIPM = (double)ConfigParameters["xTolIPM"];
