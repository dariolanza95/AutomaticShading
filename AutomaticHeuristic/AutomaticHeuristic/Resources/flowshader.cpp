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

    /*if(glm::any(glm::isnan(flow_normal)))
        data[0] = data[1] = data[2] = data[3] = 0;
    else    {
        for(int k=0;k<3;k++)
            data[k] = flow_normal[k];
        data[3] = lic_val;

    }*/
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


/*
    int _num_variables = 4; //3 from the flow_normal and 1 from lic_val

    //*variables_names = new char*[4];
   /* int** matrix = new int*[rows];
    for (int i = 0; i < rows; ++i)
        matrix[i] = new int[cols];

    char** temp_type = new char*[4];
    //char** temp_variables = new char*[4];
    std::vector<char*> temp_variables(4);
    for(int i = 0;i<4;i++){
        temp_type[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }
    std::stringstream strm;
    for(int i = 0;i<4;i++)  {
        strm<<"shader_parameter_"<<i;
        temp_variables[i] = AutomaticShaders::Utils::fromStringToChar(strm.str());

        strm.str("");
    }

    types = &temp_type;
    *variables_names = &temp_variables[0];

    for(int i = 0;i<4;i++)  {
        //std::cout<<" "<<*types[i]<<std::endl;
        std::cout<<" "<<temp_variables[i]<<std::endl;
        //*types[i] = new char[];
        //*types[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }

    *num_variables = _num_variables;

  //    *types =    new char*;
  //  *types[0] = new char;
  //    *types[0] = AutomaticShaders::Utils::fromStringToChar("float");
/*
    *variables_names = new char*[_num_variables];
    for(int i = 0;i<_num_variables-1;i++)
    {
        *variables_names[i] = new char;
        *variables_names[i] = AutomaticShaders::Utils::fromStringToChar("shader_parameter_");
    }
*/


    //*variables_names =    new char*;
    //*variables_names[0] = new char;
    //*variables_names[0] = AutomaticShaders::Utils::fromStringToChar("shader_parameter_0");

}
