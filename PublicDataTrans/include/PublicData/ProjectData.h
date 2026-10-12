#pragma once
#include <string>

struct ProjectData
{
    std::string projectName;

    // Absolute Path
    std::string modelPath;
    std::string ControlPointPath;
    std::string trajectoryPath;
    std::string slicePath;
    std::string SprayDepositionDataPath;

};