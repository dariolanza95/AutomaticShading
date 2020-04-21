#include "materialshader.h"

MaterialShader::MaterialShader(int id): AShader(id)
{

}


MaterialShader::MaterialShader(int id, float confidence, float hardness) : AShader(id,confidence),hardness(hardness)
{
}

void MaterialShader::getSerializedData(float** data)
{

    //*data = (float *)malloc(sizeof(float));

    **data = hardness;

}

void MaterialShader::allocateData(float **data){
    *data = (float * )new float;
}

void MaterialShader::getSerializedTypes(char*** types, char*** variables_names,int* num_variables)
{

    *types =    new char*;
    *types[0] = new char;
    *types[0] = AutomaticShaders::Utils::fromStringToChar("float");

    *variables_names =    new char*;
    *variables_names[0] = new char;
    *variables_names[0] = AutomaticShaders::Utils::fromStringToChar("shader_parameter_0");

    *num_variables = 1;
}
