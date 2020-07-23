#include "materialshader.h"

MaterialShader::MaterialShader(int id): AShader(id)
{

}


MaterialShader::MaterialShader(int id, float confidence, float hardness) : AShader(id,confidence),hardness(hardness)
{
}


std::string MaterialShader::getShaderName(){
    return "MaterialShader";
}

void MaterialShader::getSerializedData(std::vector<float> &data)
{
    data[0] = hardness;
}

void MaterialShader::getCloudPathName(std::string& path){
    std::string temp =std::string("../../Data/pointcloud_MaterialShader");
            //std::string("../../Data/pointcloud_MaterialShader");
    path = temp;}

void MaterialShader::allocateData(std::vector<float> &data){
    data.resize(1);
}

void MaterialShader::getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)
{
    std::cout<<"Evvov, you shouldnt be heve!!!"<<std::endl;
    types.resize(1);
    types[0] = AutomaticShaders::Utils::fromStringToChar("float");

    var_names.reserve(1);
    var_names[0] = AutomaticShaders::Utils::fromStringToChar("shader_parameter_0");

    num_variables = 1;
}
