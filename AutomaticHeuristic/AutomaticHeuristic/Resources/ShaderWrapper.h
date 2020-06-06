#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>
#include <glm/vec3.hpp>
#include <map>
#include <boost/any.hpp>
#include <queue>
#include "Ashader.h"
//#include "shaderparameter.h"

class ShadersWrapper
{
    glm::vec3 _vector;
    std::vector<AShader*> list_of_shaders;

public:
 //   static ShadersWrapper* interpolate(ShadersWrapper* sw1,ShadersWrapper* sw2,float d);
    ShadersWrapper();
    void AddShaderParameters(AShader* shader);
    void GetListOfShaders(std::vector<AShader*>& list);
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
