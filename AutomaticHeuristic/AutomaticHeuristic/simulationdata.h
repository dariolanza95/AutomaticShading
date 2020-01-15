#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <exception>
using namespace std;
class SimulationData
{
    public:
        map<string,float> _map;
        SimulationData(map<string,float> map);

        SimulationData(vector<string> nameVariables, const string& str);

};
#endif // SIMULATIONDATA_H
