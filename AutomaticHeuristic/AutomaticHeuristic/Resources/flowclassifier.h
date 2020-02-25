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
//using namespace  OpenMesh;
class FlowClassifier : public AClassifier
{
    VertexEditTag _vertex_edit_tag;
    MyMesh _mesh;
    int _shader_parameter_size;


    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle,ShaderParameters*> ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);

    map<MyMesh::VertexHandle,glm::vec3> selectFlowVertices();
    map<MyMesh::VertexHandle,ShaderParameters*> DebugFunction(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    void RefineShaderParameters(MyMesh::VertexHandle vertex,ShaderParameters *shader_parameters);
public:
    VertexEditTag GetVertexEditTag();
    FlowClassifier(MyMesh mesh);
    map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices();

};

#endif // FLOWCLASSIFIER_H
