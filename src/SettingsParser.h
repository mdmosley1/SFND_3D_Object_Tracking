#ifndef SETTINGSPARSER_H
#define SETTINGSPARSER_H


#include <memory> // unique_ptr
#include <vector>
#include <fstream>


struct Params
{
    std::string detectorType;
    std::string descriptorType;
    std::string matcherType;
    std::string selectorType;
    bool bFocusOnVehicle = true;
    int normType;
    bool visualizeMatches = true;
    int cvWaitTime = 0; // amount of time to wait before closing opencv window. If 0, wait until user presses key
};

Params LoadParamsFromFile(std::string fname);


#endif /* SETTINGSPARSER_H */
