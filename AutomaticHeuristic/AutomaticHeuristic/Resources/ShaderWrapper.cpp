#include "ShaderWrapper.h"


ShadersWrapper::ShadersWrapper() {}
/*ShaderParameters::ShaderParameters(ShaderParameters& sp) : _id(sp._id)
{
_list = sp._list;
_vector = sp._vector;

}*/

/*
static ShadersWrapper* ShadersWrapper::interpolate(ShadersWrapper* sw1,ShadersWrapper* sw2,float d){
    //for
    //sw1->list_of_shaders
return sw1;
}
*/


void ShadersWrapper::GetListOfShaders(std::vector<std::shared_ptr<AShader>>& list)
{

    //better to sort them
    list = list_of_shaders;
}

void ShadersWrapper::AddShaderParameters(std::shared_ptr<AShader> sp )
{
    list_of_shaders.push_back(sp);
}
/*
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
*/
