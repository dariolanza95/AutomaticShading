#include "sedimentationshader.h"

SedimentationShader::SedimentationShader(int id, float confidence,float material_id) : AShader(id,confidence),material_id(material_id)
{}

void SedimentationShader::allocateData(std::vector<float> &data){
    data.resize(1);
}

void SedimentationShader::getSerializedData(std::vector<float>& data)
{
    /*for(int k = 0;k<3;k++)
        data[k] = flow_normal[k];
    data[3] = lic_val;*/
    data[0] = material_id;
}
std::string SedimentationShader::getShaderName(){
    return "SedimentationShader";
}

void SedimentationShader::getCloudPathName(std::string& path){
    std::string temp = std::string("pointcloud_SedimentationShader");
            //std::string("../../Data/pointcloud_SedimentationShader");
    path = temp;
}
void SedimentationShader::getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)
{
    types.resize(1);
    var_names.resize(1);
    num_variables = 1;
    for(int i = 0;i<1;i++){
        types[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }

    std::stringstream strm;
    for(int i = 0;i<4;i++){
        strm<<"shader_parameter_"<<i;
        var_names[i] = AutomaticShaders::Utils::fromStringToChar(strm.str());
        strm.str("");
    }
}
