#pragma once
#include "sources/BatchTestutils.h"
#include "sources/EnergyOptimization/EnergyIftopSpec.h"
#include "sources/EnergyOptimization/EnergyOptimize.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/EnergyOptimization/OptimizeSA.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/RTA/RTA_Nasri19.h"

namespace rt_num_opt {

inline std::string GetResFileName(const std::string &pathDataset,
                                  const std::string &file) {
    if (optimizerType == 5) {
        return pathDataset + file + "_SA_Res.txt";
    } else if (optimizerType == 6) {
        return pathDataset + file + "_IPM_Res.txt";
    } else if (elimIte > 0) {
        return pathDataset + file + "_elim_approx_Res.txt";
    } else {
        return pathDataset + file + "_not_elim_Res.txt";
    }
}
void WriteToResultFile(std::string resFile, double res, double timeTaken) {
    std::ofstream outfileWrite;
    outfileWrite.open(resFile, std::ios_base::app);
    outfileWrite << res << std::endl;
    outfileWrite << timeTaken << std::endl;
    outfileWrite.close();
}
void WriteToResultFile(const std::string &pathDataset, const std::string &file,
                       double res, double timeTaken) {
    std::string resFile = GetResFileName(pathDataset, file);
    WriteToResultFile(resFile, res, timeTaken);
}

std::pair<double, double> ReadFromResultFile(std::string resFile) {
    std::ifstream cResultFile(resFile.data());
    double timeTaken = 0, res = 0;
    cResultFile >> res >> timeTaken;
    cResultFile.close();
    return std::make_pair(res, timeTaken);
}
std::pair<double, double> ReadFromResultFile(const std::string &pathDataset,
                                             const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    return ReadFromResultFile(resFile);
}

bool VerifyResFileExist(std::string resFile) {
    std::ifstream myfile;
    myfile.open(resFile);
    if (myfile) {
        return true;
    } else {
        return false;
    }
}
bool VerifyResFileExist(const std::string &pathDataset,
                        const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    return VerifyResFileExist(resFile);
}

}  // namespace rt_num_opt