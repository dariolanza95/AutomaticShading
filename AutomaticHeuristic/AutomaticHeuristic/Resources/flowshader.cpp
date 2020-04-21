#include "flowshader.h"

FlowShader::FlowShader(int id) : AShader(id)
{

}

FlowShader::FlowShader(int id,float confidence,glm::vec3 flow_normal) : AShader(id,confidence),flow_normal(flow_normal)
{
    lic_val = 0;
}




void FlowShader::getSerializedData(float** data)
{
   /* for(int k = 0;k<3;k++)
        data[k] = flow_normal[k];
    data[4] = lic_val;*/
}

void FlowShader::allocateData(float **data)
{
    *data = (float * )new float;
}

void FlowShader::getSerializedTypes(char*** types, char*** variables_names,int *num_variables)
{

    int _num_variables = 4; //3 from the flow_normal and 1 from lic_val

    //*variables_names = new char*[4];
   /* int** matrix = new int*[rows];
    for (int i = 0; i < rows; ++i)
        matrix[i] = new int[cols];
*/
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
