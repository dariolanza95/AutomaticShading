#include "featuresfinder.h"

FeaturesFinder::FeaturesFinder(MyMesh mesh): _mesh(mesh){}

MyMesh  FeaturesFinder::Find()
{

    //angle of repose is usually between 33-37 degreee depending on the rock type
    float angle = 10;
    float treshold = 3;
     map<MyMesh::VertexHandle,ShaderParameters*> selected_faces;
     InitializerSimulationData();
     AClassifier *sc = new ScreeClassifier(_mesh,angle,treshold);
     selected_faces = sc->ClassifyVertices();
     UpdateSimulationData(selected_faces);
     selected_faces.clear();
     AClassifier *rc = new RiverClassifier(_mesh,75,20,5,34,-35);
     selected_faces = rc->ClassifyVertices();
     UpdateSimulationData(selected_faces);
     AClassifier *fc = new FlowClassifier(_mesh);
     selected_faces = fc->ClassifyVertices();
     UpdateSimulationData(selected_faces);
     //RiverClassifierTester rct;
     //rct.Test();
    return _mesh;
}


vector<VertexEditTag> FeaturesFinder::GetVertexEditTags()
{
    return _vertex_edit_tags;
}


void FeaturesFinder::UpdateSimulationData(map<MyMesh::VertexHandle,ShaderParameters*> selected_vertices)
{
    auto shader_parameters_property = getOrMakeProperty<VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");
    for (auto const& x : selected_vertices)
    {
        MyMesh::VertexHandle vertex_handle = x.first;
        ShaderParameters* shader_parameter = x.second;

        shader_parameters_property[vertex_handle] = shader_parameter;

    }
}

void FeaturesFinder::InitializerSimulationData()
{
auto shader_parameters_data_wrapper= getOrMakeProperty<VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");

    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    for(vertex_iterator = _mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        ShaderParameters* shader_parameter = new ShaderParameters(0,10);
        shader_parameters_data_wrapper[vertex_iterator] = shader_parameter;
    }
}
