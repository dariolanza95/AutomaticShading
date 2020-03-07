#include "ShaderParameters.h"

ShaderParameters::ShaderParameters(int shaderId,int size) : _list(size), _id(shaderId)
{
    for(uint i = 0;i<_list.size();i++)
    {
        _list[i] = 0;
    }
    _vector = glm::vec3(0,0,0);
}
ShaderParameters::ShaderParameters() : _list(0), _id(0){}

float ShaderParameters::getValue(int index)
{
    return _list[index];
}

void ShaderParameters:: setValue (int index,float value)
{
    if(index<_list.size())
    {
        _list[index] = value;
    }
    else
    {
        _list.resize(2*_list.capacity());
        _list[index] = value;
    }
}

glm::vec3 ShaderParameters::getVector() {return _vector;}
void ShaderParameters::setVector(glm::vec3 vec) {_vector = vec; }

int ShaderParameters::getId() {return _id;}
