#ifndef FLOWSHADER_H
#define FLOWSHADER_H

#include "Ashader.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
#include <strstream>
#include "flowshader.h"
#include "utils.h"
class FlowShader : public AShader
{
    glm::vec3 flow_normal;
    float lic_val;
public:
    FlowShader(int id,float confidence,glm::vec3 flow_normal);
    FlowShader(int id,float confidence,glm::vec3 flow_normal,float lic_value);

    FlowShader();
    FlowShader(int id);
    glm::vec3 GetFlowNormal();
    float GetLicValue();
    void SetLicValue(float new_val);
    void getOutputCloudPath(std::string& path);
    void allocateData(std::vector<float> &data);
    void getSerializedData(std::vector<float> &data);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);

};

#endif // FLOWSHADER_H
