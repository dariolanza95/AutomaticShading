#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>
#include <glm/vec3.hpp>
class ShaderParameters
{
   const int  _id ;

    glm::vec3 _vector;
public:

    ShaderParameters();
    //ShaderParameters(ShaderParameters &sp);

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
