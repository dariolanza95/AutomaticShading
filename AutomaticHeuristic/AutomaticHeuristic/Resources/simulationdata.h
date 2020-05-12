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


enum class SimulationDataEnum{vegetation,river,hardness,flow_normal,initial_sedimentation_point,sedimentation_history};

class SimulationData
{
        std::map<std::string, SimulationDataEnum> SimulationDataEnummap = {
            {"flow_normal", SimulationDataEnum::flow_normal},
            {"hardness", SimulationDataEnum::hardness},
            {"sedimentation_history", SimulationDataEnum::sedimentation_history},
            {"initial_sedimentation_point", SimulationDataEnum::initial_sedimentation_point}
            };

        std::map<SimulationDataEnum,float> map_of_floats;
        std::map<SimulationDataEnum,glm::vec3> map_of_vectors;
        std::map<SimulationDataEnum,std::vector<float>> map_of_lists;
        void readLine(const std::string line );

    public:
        SimulationData(const std::string str);
        SimulationData();
        SimulationData( std::map<SimulationDataEnum,float> map_of_floats,
                                        std::map<SimulationDataEnum,glm::vec3> map_of_vectors,
                                        std::map<SimulationDataEnum,std::vector<float>> map_of_lists);
        SimulationData* Interpolate(SimulationData* sd1,float t);
        void getData(SimulationDataEnum data_enum, float& data);
        void getData(SimulationDataEnum data_enum, glm::vec3& data);
        void getData(SimulationDataEnum data_enum, std::vector<float>& data);
        //SimulationData* operator /(SimulationData* sd);
        //SimulationData* operator +(SimulationData* sd);

};
#endif // SIMULATIONDATA_H
