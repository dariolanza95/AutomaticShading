#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <exception>
#include <boost/any.hpp>
#include <glm/vec3.hpp>
using namespace std;
class SimulationData
{
    public:
        map<string,boost::any> _map;
        SimulationData(map<string,boost::any> map);

        SimulationData(vector<string> nameVariables, const string& str);

};
#endif // SIMULATIONDATA_H
