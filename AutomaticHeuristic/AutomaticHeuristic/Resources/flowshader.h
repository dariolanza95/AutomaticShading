#ifndef FLOWSHADER_H
#define FLOWSHADER_H

#include "Ashader.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
#include "flowshader.h"
#include "utils.h"
class FlowShader : public AShader
{
    glm::vec3 flow_normal;
    float lic_val;
public:
    FlowShader(int i,float confidence,glm::vec3 flow_normal);
    FlowShader();
    FlowShader(int id);

    void allocateData(float** data);
    void getSerializedData(float** data);
    void getSerializedTypes(char ***types, char ***variables_names, int *num_variables);

};

#endif // FLOWSHADER_H
