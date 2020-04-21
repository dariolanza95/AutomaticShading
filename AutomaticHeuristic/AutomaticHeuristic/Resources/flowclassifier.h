#ifndef FLOWCLASSIFIER_H
#define FLOWCLASSIFIER_H
#include "aclassifier.h"
#include "VertexEditTag.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
#include "flowshader.h"
//using namespace  OpenMesh;
class FlowClassifier : public AClassifier
{
    VertexEditTag _vertex_edit_tag;
    int _shader_parameter_size;
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle,AShader*> ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    map<MyMesh::VertexHandle,glm::vec3> selectFlowVertices();
    //map<MyMesh::VertexHandle,ShaderParameters*> DebugFunction(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    //void LIC2(float box_length,float frequency,float step_size,MyMesh mesh);
    //void RefineShaderParameters(MyMesh::VertexHandle vertex,ShaderParameters *shader_parameters);
public:
    VertexEditTag GetVertexEditTag();
    FlowClassifier(MyMesh mesh);
    AShader* GetShader();
    map<MyMesh::VertexHandle,AShader*> ClassifyVertices();

};

#endif // FLOWCLASSIFIER_H
