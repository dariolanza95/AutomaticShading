#ifndef MATERIALSHADER_H
#define MATERIALSHADER_H
#include "AShader.h"
#include "utils.h"
#include <stdlib.h>
#include <iostream>
#include <string>

class MaterialShader : public AShader
{
    float hardness;
public:
    MaterialShader();
    MaterialShader(int _id);

    MaterialShader(int _id,float _confidence,float hardness);

    std::string getShaderName();
    void getCloudPathName(std::string& path);
    void allocateData(std::vector<float> &data);
    void getSerializedData(std::vector<float> &data);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);
};

#endif // MATERIALSHADER_H
