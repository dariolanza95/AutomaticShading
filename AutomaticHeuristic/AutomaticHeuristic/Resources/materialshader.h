#ifndef MATERIALSHADER_H
#define MATERIALSHADER_H
#include "Ashader.h"
#include "utils.h"
#include <stdlib.h>
#include <iostream>
#include <string>

class MaterialShader : public AShader
{
    float hardness;
public:
    MaterialShader();
    MaterialShader(int id);

    MaterialShader(int id,float confidence,float hardness);

    void getOutputCloudPath(std::string& path);
    void allocateData(std::vector<float> &data);
    void getSerializedData(std::vector<float> &data);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);
};

#endif // MATERIALSHADER_H
