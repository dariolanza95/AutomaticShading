#ifndef FLOWSHADER_H
#define FLOWSHADER_H

#include "AShader.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
#include <strstream>
#include "utils.h"
class FlowShader : public AShader
{
    glm::vec3 flow_normal;
    float lic_val;
public:
    FlowShader(int _id,float _confidence,glm::vec3 flow_normal);
    FlowShader(int _id,float _confidence,glm::vec3 flow_normal,float lic_value);

    FlowShader();
    FlowShader(int _id);
    glm::vec3 GetFlowNormal();
    float GetLicValue();
    void SetLicValue(float new_val);
    std::string getShaderName();
    void getCloudPathName(std::string& path);
    void allocateData(std::vector<float> &data);
    void getSerializedData(std::vector<float> &data);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);

};

#endif // FLOWSHADER_H
