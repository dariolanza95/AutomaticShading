#include "ShaderParameters.h"

ShaderParameters::ShaderParameters(int shaderId,int size) : _list(size), _id(shaderId){}
ShaderParameters::ShaderParameters() : _list(0), _id(0){}

float ShaderParameters::getValue(int index)
{
    return _list[index];
}

void ShaderParameters:: setValue (int index,float value)
{
    _list[index] = value;
}

glm::vec3 ShaderParameters::getVector() {return _vector;}
void ShaderParameters::setVector(glm::vec3 vec) {_vector = vec; }

int ShaderParameters::getId() {return _id;}
