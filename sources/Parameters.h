#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

cv::FileStorage ConfigParameters("/home/zephyr/Programming/Energy_Opt_NLP/sources/parameters.yaml", cv::FileStorage::READ);

using namespace std;
// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
const int TASK_NUMBER  = 5;
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];

const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];

const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
const double weightEnergy = (double)ConfigParameters["weightEnergy"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];