#include "flowclassifier.h"

FlowClassifier::FlowClassifier(MyMesh mesh) : AClassifier(),_mesh(mesh),_shader_parameter_size(1)
{
    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
}


map<MyMesh::VertexHandle,ShaderParameters*> FlowClassifier::ClassifyVertices()
{
    map<MyMesh::VertexHandle,ShaderParameters*> selected_vertices;
    auto flow_vertices = selectFlowVertices();
    selected_vertices =  ComputeShaderParameters(flow_vertices);
    return selected_vertices;
}

map<MyMesh::VertexHandle,ShaderParameters*> FlowClassifier::ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices)
{
    glm::vec3 up(0,1,0);
    map<MyMesh::VertexHandle,ShaderParameters*> map;
    for(pair<MyMesh::VertexHandle,glm::vec3> entry : flow_vertices)
    {

           //the shaders requires the orthogoanl vector
          glm::vec3 orthogonal_vector = glm::cross(entry.second,up);

          //for(int i=0;i<3;i++)
          //  orthogonal_vector[i] = orthogonal_vector[i]>0.01 ? orthogonal_vector[i]:0;
          orthogonal_vector = glm::normalize (orthogonal_vector);
          glm::vec3 final_vector = glm::cross(entry.second,orthogonal_vector);

          ShaderParameters* shader_parameters = new ShaderParameters(_id,_shader_parameter_size);
          shader_parameters->setVector(final_vector);
          if(glm::any(glm::isnan( shader_parameters->getVector())))
              cout<<"come here sir there is an error ";

          map.insert(make_pair(entry.first,shader_parameters));
    }
return map;
}




map<MyMesh::VertexHandle,glm::vec3> FlowClassifier::selectFlowVertices()
{
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<MyMesh::VertexHandle,glm::vec3> flow_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //normalize it!
        glm::vec3 normal = boost::any_cast<glm::vec3>(sd->_map.at("normalFlow"));

        if(normal[0] != 0 || normal[1] != 0 || normal[2] != 0)
        {
            flow_vertices.insert(make_pair(*vertex_iterator,boost::any_cast<glm::vec3>(sd->_map.at("normalFlow"))));
        }
    }
return flow_vertices;
}

