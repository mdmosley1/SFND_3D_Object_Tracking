#include "SettingsParser.h"
#include <sstream> // istringstream
#include <map> // map
#include <iostream> // cout

using namespace std;


Params LoadParamsFromFile(std::string fname)
{
    Params p;
    std::ifstream file(fname);
    std::stringstream buffer;
    buffer << file.rdbuf();

    std::map<std::string, std::string> paramsMap;

    std::string line;
    while( std::getline(buffer, line) )
    {
        // skip line if it starts with #
        if (line.find('#') == 0)
            continue;

        std::istringstream is_line(line);
        std::string key;
        if( std::getline(is_line, key, '=') )
        {
            std::string value;
            if( std::getline(is_line, value) )
                paramsMap.emplace(key,value);
        }
    }

    // print params
    cout << "######### LOADED PARAMS ########" << "\n";
    for (auto it = paramsMap.begin(); it != paramsMap.end(); ++it)
        cout << it->first << " : " << it->second << "\n";
    cout << "################################" << "\n\n";

    p.detectorType = paramsMap["detectorType"];
    p.descriptorType = paramsMap["descriptorType"];
    p.matcherType = paramsMap["matcherType"];
    p.selectorType = paramsMap["selectorType"];
    p.bFocusOnVehicle = std::stoi(paramsMap["bFocusOnVehicle"]);
    p.normType = std::stoi(paramsMap["normType"]);
    p.visualizeMatches = std::stoi(paramsMap["visualizeMatches"]);
    return p;
}
