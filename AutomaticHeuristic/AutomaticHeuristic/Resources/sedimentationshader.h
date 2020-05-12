#ifndef SEDIMENTATIONSHADER_H
#define SEDIMENTATIONSHADER_H

#include "Ashader.h"
#include "utils.h"
#include <sstream>

class SedimentationShader : public AShader
{
    float material_id;

public:
    SedimentationShader(int id, float confidence,float material_id);
    void getSerializedData(std::vector<float>& data);
    void allocateData(std::vector<float> &data);
    std::string getShaderName();
    void getCloudPathName(std::string& path);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);
};

#endif // SEDIMENTATIONSHADER_H
