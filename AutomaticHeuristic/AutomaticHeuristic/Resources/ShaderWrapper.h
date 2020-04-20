#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>
#include <glm/vec3.hpp>
#include <map>
#include <boost/any.hpp>
#include <queue>

#include "shaderparameter.h"

class ShadersWrapper
{


    std::map<std::string, ShaderParametersEnum> ShaderParameterOutputEnumMap= {
        {"flow_normal", ShaderParametersEnum::flow_normal},
        {"flow_normal", ShaderParametersEnum::flow_normal},
        {"hardness",    ShaderParametersEnum::hardness}
        };

    glm::vec3 _vector;
    std::vector<ShaderParameters> list_of_shaders;

public:
    ShadersWrapper();
    void AddShaderParameters(ShaderParameters sp);
    void GetListOfShaders(std::vector<ShaderParameters>& list);
    //ShadersWrapper(int id);
    //int getId();
    //std::vector<float> _list;
    //int getParametersListSize();
    //void setVector(glm::vec3);
    //glm::vec3 getVector();
    //float getValue(int index);
    //void setValue(int index,float value);

};



#endif // SHADERPARAMETERS_H
