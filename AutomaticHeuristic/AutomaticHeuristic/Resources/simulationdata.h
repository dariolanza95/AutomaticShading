#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <exception>
#include <glm/vec3.hpp>
#include <vector>
#include <memory>
#include "../Graphics/exceptionclass.h"


enum class SimulationDataEnum{stack_id,air_pressure,vegetation,river,sed_level,hardness,flow_normal,initial_sedimentation_point,material_stack_height, sedimentation_history,actual_point};

class SimulationData
{
        std::map<std::string, SimulationDataEnum> SimulationDataEnummap = {
            {"flow_normal", SimulationDataEnum::flow_normal},
            {"material_stacks_id", SimulationDataEnum::stack_id},
            {"pressure",SimulationDataEnum::air_pressure},
            {"hardness", SimulationDataEnum::hardness},
            {"sediment_value", SimulationDataEnum::sedimentation_history},
            {"initial_sedimentation_point", SimulationDataEnum::initial_sedimentation_point},
            {"actual_point",SimulationDataEnum::actual_point},
            {"sed_level",SimulationDataEnum::sed_level},
            {"material_stacks_height",SimulationDataEnum::material_stack_height},
            };

        std::map<SimulationDataEnum,float> map_of_floats;
        std::map<SimulationDataEnum,glm::vec3> map_of_vectors;
        std::map<SimulationDataEnum,std::vector<float>> map_of_lists;
        void readLine(const std::string line );

    public:
        SimulationData(const std::string str);
        SimulationData();
        ~SimulationData();
        SimulationData( std::map<SimulationDataEnum,float> map_of_floats,
                                        std::map<SimulationDataEnum,glm::vec3> map_of_vectors,
                                        std::map<SimulationDataEnum,std::vector<float>> map_of_lists);
        std::shared_ptr<SimulationData> Interpolate(std::shared_ptr<SimulationData> sd1, float t);
        void getData(SimulationDataEnum data_enum, float& data);
        void getData(SimulationDataEnum data_enum, glm::vec3& data);
        void getData(SimulationDataEnum data_enum, std::vector<float>& data);
        //SimulationData* operator /(SimulationData* sd);
        //SimulationData* operator +(SimulationData* sd);

};
#endif // SIMULATIONDATA_H
