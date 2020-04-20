#include "ShaderWrapper.h"

ShadersWrapper::ShadersWrapper(int shaderId,int size) : _list(8), _id(shaderId)
{
    for(uint i = 0;i<_list.size();i++)
    {
        _list[i] = 0;
    }
    _vector = glm::vec3(0,0,0);
}
ShadersWrapper::ShadersWrapper() : _id(0){
    AddShaderParameters(ShaderParametersEnum::empty,0);
}
/*ShaderParameters::ShaderParameters(ShaderParameters& sp) : _id(sp._id)
{
_list = sp._list;
_vector = sp._vector;

}*/



ShadersWrapper::GetListOfShaders(std::vector<ShaderParameters>& list)
{

    //better to sort them
    list = list_of_shaders;
}

ShadersWrapper::AddShader(ShaderParameters sp )
{
    list_of_shaders.push_back(sp);
}

float ShadersWrapper::getValue(int index)
{
    return _list[index];
}


void ShadersWrapper::GetParametersMap(std::map<ShaderParametersEnum,boost::any>& parameters_map)
{
    parameters_map = map;
}

void ShadersWrapper:: setValue (int index,float value)
{
    if(index<_list.size())
    {
        _list[index] = value;
    }
    else
    {
        while(index>=_list.size())
            _list.resize(2*_list.capacity());
        _list[index] = value;
    }
}

glm::vec3 ShadersWrapper::getVector() {return _vector;}
void ShadersWrapper::setVector(glm::vec3 vec) {_vector = vec; }

int ShadersWrapper::getId() {return _id;}
