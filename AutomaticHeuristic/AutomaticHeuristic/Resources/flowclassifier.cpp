#include "flowclassifier.h"

FlowClassifier::FlowClassifier(MyMesh mesh) : AClassifier(mesh),_shader_parameter_size(3)
{
    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
    std::vector<int> temp;
    temp.push_back(0);
    temp.push_back(1);
    temp.push_back(2);

    _vertex_edit_tag = VertexEditTag(temp);
}

 VertexEditTag FlowClassifier::GetVertexEditTag()
 {
     return _vertex_edit_tag;
 }

map<MyMesh::VertexHandle,ShaderParameters*> FlowClassifier::ClassifyVertices()
{

    map<MyMesh::VertexHandle,ShaderParameters*> selected_vertices;
    auto flow_vertices = selectFlowVertices();
    selected_vertices =  ComputeShaderParameters(flow_vertices);
    return selected_vertices;
}


//void FlowClassifier::RefineShaderParameters(MyMesh::VertexHandle vertex,ShaderParameters* shader_parameters)
//{
//    //except for border vertices they should all have valence 6
//    int valence = _mesh.valence(vertex);
//    int i = 0;
//    for (MyMesh::VertexIHalfedgeIter vertex_half_edge_iterator = _mesh.vih_iter(vertex); vertex_half_edge_iterator.is_valid(); ++vertex_half_edge_iterator)
//    {
//        i++;
//        MyMesh::Halfedge half_edge = _mesh.halfedges(*vertex_half_edge_iterator);
//
//        OpenMesh::Concepts::MeshItems::HalfedgeT< Refs_ >::face_handle 	( 		) 	const
//        MyMesh::Vertex opposite_vertex = _mesh.opposite_vertex_handle(*vertex_half_edge_iterator);
//        MyMesh::Face face = _mesh.faces(*vertex_half_edge_iterator);
//    }
//
//}

map<MyMesh::VertexHandle,ShaderParameters*> FlowClassifier::DebugFunction(map<MyMesh::VertexHandle,glm::vec3> flow_vertices )
{
    map<MyMesh::VertexHandle,ShaderParameters*> map;

    //_vertex_edit_tag.AddVertexChange();
    for(pair<MyMesh::VertexHandle,glm::vec3> entry : flow_vertices)
    {
        MyMesh::VertexFaceIter vertex_face_circulator;

        ShaderParameters* shader_parameters = new ShaderParameters(_id,1.0f);

        glm::vec3 final_vector = glm::vec3(1,0,0);
        shader_parameters->AddParameter(ShaderParametersEnum::flow_normal,final_vector);
        //shader_parameters->setValue(0,final_vector[0]);
        //shader_parameters->setValue(1,final_vector[1]);
        //shader_parameters->setValue(2,final_vector[2]);
       // RefineShaderParameters(entry.first,shader_parameters);
        map.insert(make_pair(entry.first,shader_parameters));

    }
    vector<int>   vertex_path;
    vector<float> new_values;
    VertexChanges vertex_changes();
    return        map;
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
          glm::vec3 tangent_vector = glm::cross(entry.second,orthogonal_vector);

         MyMesh::Normal openMesh_normal = _mesh.normal(entry.first);
         glm::vec3 normal = glm::vec3(openMesh_normal[0],openMesh_normal [1],openMesh_normal[2]);
         float res =   fabs(dot(normal,orthogonal_vector));
         float res_2 = fabs(dot(normal,tangent_vector));
         glm::vec3 final_vector = res > res_2 ? tangent_vector : orthogonal_vector;
   //     final_vector = tangent_vector;
         ShaderParameters* shader_parameters = new ShaderParameters(_id,1.0f);

         shader_parameters->AddParameter(ShaderParametersEnum::flow_normal,final_vector);
         //shader_parameters->setValue(0,final_vector[0]);
         //shader_parameters->setValue(1,final_vector[1]);
         //shader_parameters->setValue(2,final_vector[2]);
         //shader_parameters->setVector(tangent_vector);

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
        glm::vec3 normal;
        sd->getData(SimulationDataEnum::flow_normal,normal);
        if(normal[0] != 0 || normal[1] != 0 || normal[2] != 0)
        {
            flow_vertices.insert(make_pair(*vertex_iterator,normal));
        }
    }
return flow_vertices;
}

