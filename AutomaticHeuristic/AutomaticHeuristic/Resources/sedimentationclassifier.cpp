#include "sedimentationclassifier.h"



SedimentationClassifier::SedimentationClassifier(MyMesh mesh) : AClassifier(mesh)
{

}
map<MyMesh::VertexHandle,sedimentationData*> SedimentationClassifier::SelectSedimentationPoints(){
    map<MyMesh::VertexHandle,sedimentationData*> selected_vertices;
    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end = _mesh.vertices_end();
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator){
        SimulationData* sd  = simulation_data_wrapper[*vertex_iterator];
           std::vector<float> sediment_history;
           sd->getData(SimulationDataEnum::sedimentation_history,sediment_history);
           glm::vec3 initial_sedimentation_point;
           sd->getData(SimulationDataEnum::initial_sedimentation_point,initial_sedimentation_point);
           if(sediment_history.size()>0 && glm::any(glm::isnan(initial_sedimentation_point))){
            sedimentationData* sd = new sedimentationData(initial_sedimentation_point,sediment_history);
            selected_vertices.insert(std::make_pair(vertex_iterator,sd));
           }
    }

return selected_vertices;
}
/*
SedimentationClassifier::CreatePointCloud(){

}

SedimentationClassifier::ComputeSedimentationParameters(){

}*/

 map<MyMesh::VertexHandle,AShader*> SedimentationClassifier::ClassifyVertices(){

auto selected_vertices = SelectSedimentationPoints();

     map<MyMesh::VertexHandle,AShader*> res;
 return res;
}
