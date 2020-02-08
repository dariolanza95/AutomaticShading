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

int ShaderParameters::getId() {return _id;}
