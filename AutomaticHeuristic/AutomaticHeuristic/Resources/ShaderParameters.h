#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>
#include <glm/vec3.hpp>
#include <map>

enum class ShaderParametersEnum{flow_normal,hardness};
class ShaderParameters
{
   const int  _id ;

    std::map<std::string, ShaderParametersEnum> ShaderParametersEnummap= {
        {"flow_normal", ShaderParametersEnum::flow_normal},
        {"hardness",    ShaderParametersEnum::hardness}
        };

    std::map<ShaderParametersEnum,float> map_of_floats;
    std::map<ShaderParametersEnum,glm::vec3> map_of_vectors;
    glm::vec3 _vector;


public:

    ShaderParameters();

    ShaderParameters(int id,int size);
    int getId();
    std::vector<float> _list;
    int getParametersListSize();
    void setVector(glm::vec3);
    glm::vec3 getVector();
    float getValue(int index);
    void setValue(int index,float value);

};



#endif // SHADERPARAMETERS_H
