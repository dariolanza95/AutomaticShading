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
#include <./../Noise/FastNoise/FastNoise.h>
#include "subdividerandinterpolator.h"

class FlowClassifier : public AClassifier
{
    VertexEditTag _vertex_edit_tag;
    int _shader_parameter_size;
    map<MyMesh::VertexHandle,AShader*> LIC(float box_length, float step_size, float frequency);
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , FlowShader*>::type, MyMesh> flow_shader_temp_propery;
    void ContrastEnhancement(map<MyMesh::VertexHandle,AShader*> map,std::vector<int> Pdf);

    void ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    map<MyMesh::VertexHandle,glm::vec3> selectFlowVertices();
    void TemporaryUpdate(map<MyMesh::VertexHandle,FlowShader*> selected_vertices);

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
