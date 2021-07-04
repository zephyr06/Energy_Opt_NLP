#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

cv::FileStorage ConfigParameters("/home/lab/Programming/Energy_Opt_NLP/sources/parameters.yaml", cv::FileStorage::READ);

using namespace std;
// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
// int TASK_NUMBER_DYNAMIC = 10;
const int TASK_NUMBER = 3;
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];

const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];

const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
double weightEnergy = (double)ConfigParameters["weightEnergy"];

const int weightEnergyMinOrder = (int)ConfigParameters["weightEnergyMinOrder"];
const int weightEnergyMaxOrder = (int)ConfigParameters["weightEnergyMaxOrder"];

const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double punishmentFrequency = (double)ConfigParameters["punishmentFrequency"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];

const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const int granularityInBF = (int)ConfigParameters["granularityInBF"];
const double toleranceInOuterLoop = (double)ConfigParameters["toleranceInOuterLoop"];
