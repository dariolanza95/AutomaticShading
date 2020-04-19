#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <exception>
#include <glm/vec3.hpp>
#include <vector>
#include "../Graphics/exceptionclass.h"


enum class SimulationDataEnum{vegetation,river,hardness,flow_normal};

class SimulationData
{
        std::map<std::string, SimulationDataEnum> SimulationDataEnummap = {
            {"flow_normal", SimulationDataEnum::flow_normal},
            {"hardness", SimulationDataEnum::hardness}
            };

        std::map<SimulationDataEnum,float> map_of_floats;
        std::map<SimulationDataEnum,glm::vec3> map_of_vectors;
        void readLine(const std::string line );

    public:
        SimulationData(const std::string str);
        void getData(SimulationDataEnum data_enum, float& data);
        void getData(SimulationDataEnum data_enum, glm::vec3& data);


};
#endif // SIMULATIONDATA_H
