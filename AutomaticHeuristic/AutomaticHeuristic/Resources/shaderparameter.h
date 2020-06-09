#ifndef SHADERPARAMET_H
#define SHADERPARAMET_H
#include <glm/vec3.hpp>
#include <map>
#include <boost/any.hpp>
#include "AShader.h"
enum class ShaderParametersEnum{empty,flow_normal,hardness,river,LIC};
enum class BlendingMode{Add,Mix,Overlap};
/*

class ShaderParameters
{

    std::map<ShaderParametersEnum,boost::any> map;
    const int  id ;
    float confidence;
public:
    ShaderParameters(int id,float confidence);
    ShaderParameters(int id);
    float GetConfidence();
    void SetConfidence(float new_confidence);
    int GetId();
    void virtual Serialize();

    //template <typename T>
    //void AddParameter(ShaderParametersEnum data_name,T data_value);
    //template <typename T>
    //void GetParameter(ShaderParametersEnum data_name,T& data_value);
    //void GetParameters(std::map<ShaderParametersEnum,boost::any>& data_value);

};

/*
template<typename T>
void ShaderParameters::AddParameter(ShaderParametersEnum data_name,T data_value)
{

    map.insert(std::pair<ShaderParametersEnum,boost::any>(data_name,data_value));
}

template<typename T>
void ShaderParameters::GetParameter(ShaderParametersEnum data_name,T& data_returned_value)
{
    if(map.count(data_name)>0)
    {
        data_returned_value =boost::any_cast<T> (map[data_name]);
    }
}*/

#endif // SHADERPARAMET_H
