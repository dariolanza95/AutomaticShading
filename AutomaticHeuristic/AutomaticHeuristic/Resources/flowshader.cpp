#include "flowshader.h"

FlowShader::FlowShader(int id) : AShader(id)
{

}

FlowShader::FlowShader(int id, float confidence, glm::vec3 flow_normal) : AShader(id,confidence),flow_normal(flow_normal),lic_val(-1)
{}

FlowShader::FlowShader(int id, float confidence, glm::vec3 flow_normal, float lic_value) : AShader(id,confidence),flow_normal(flow_normal),lic_val(lic_value)
{}

void FlowShader::allocateData(std::vector<float> &data){
    data.resize(4);
}

void FlowShader::getSerializedData(std::vector<float>& data)
{
    for(int k = 0;k<3;k++)
        data[k] = flow_normal[k];
    data[3] = lic_val;
}
std::string FlowShader::getShaderName(){
    return "FlowShader";
}

void FlowShader::getCloudPathName(std::string& path){
    std::string temp = std::string("pointcloud_FlowShader");
            //std::string("../../Data/pointcloud_FlowShader");
    path = temp;
}
glm::vec3 FlowShader::GetFlowNormal(){return flow_normal;}
float FlowShader::GetLicValue(){return lic_val;}
void FlowShader::SetLicValue(float new_val){lic_val=new_val;}
void FlowShader::getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)
{
    types.resize(4);
    var_names.resize(4);
    num_variables = 4;
    for(int i = 0;i<4;i++){
        types[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }

    std::stringstream strm;
    for(int i = 0;i<4;i++){
        strm<<"shader_parameter_"<<i;
        var_names[i] = AutomaticShaders::Utils::fromStringToChar(strm.str());
        strm.str("");
    }
}
