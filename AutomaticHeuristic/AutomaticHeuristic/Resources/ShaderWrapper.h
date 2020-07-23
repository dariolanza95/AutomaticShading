#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>
#include <glm/vec3.hpp>
#include <map>
#include <boost/any.hpp>
#include <queue>
#include "AShader.h"
#include <memory>
//#include "shaderparameter.h"

class ShadersWrapper
{
    glm::vec3 _vector;
    std::vector<std::shared_ptr<AShader>> list_of_shaders;

public:
 //   static ShadersWrapper* interpolate(ShadersWrapper* sw1,ShadersWrapper* sw2,float d);
    ShadersWrapper();
    void AddShaderParameters(std::shared_ptr<AShader> shader);
    void GetListOfShaders(std::vector<std::shared_ptr<AShader>>& list);
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
