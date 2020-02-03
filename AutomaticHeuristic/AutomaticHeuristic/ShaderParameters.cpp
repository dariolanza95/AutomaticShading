#include "ShaderParameters.h"

ShaderParameters::ShaderParameters(int shaderId,int size) : _list(size), _id(shaderId)
{
}

int ShaderParameters::getValue(int index)
{
    return _list[index];
}

void ShaderParameters:: setValue (int index,float value)
{
    _list[index] = value;
}

int ShaderParameters::getId() {return _id;}
