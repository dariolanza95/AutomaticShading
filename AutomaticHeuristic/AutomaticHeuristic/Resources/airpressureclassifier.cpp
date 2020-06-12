#include "airpressureclassifier.h"
/*
AirPressureClassifier::AirPressureClassifier(MyMesh mesh): AClassifier(mesh)
{
    _shader ;
}

std::shared_ptr<AShader> AirPressureClassifier::GetShader() {return _shader;}

AirPressureClassifier::ClassifyVertices(std::vector<glm::vec3> &list_of_points, std::vector<std::shared_ptr<AShader> > &list_of_data, float &details){

    auto selected_vertices = selectAirPressureVertices();
    for(auto const entry: selected_vertices){
        MyMesh::Point p = _mesh.point(entry.first);
        list_of_points.push_back(glm::vec3(p[0],p[1],p[2]));
    }

}
map<MyMesh::VertexHandle,float> AirPressureClassifier::selectAirPressureVertices()
{
    float treshold = 0;
    map<MyMesh::VertexHandle,float> air_pressure_vertices;
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
   std::shared_ptr<SimulationData> sd = simulation_data_wrapper[*vertex_iterator];
   if(sd==nullptr)
       continue;
        float air_pressure_vertices_data;
        sd->getData(SimulationDataEnum::air_pressure,air_pressure_vertices_data);
        if(air_pressure_vertices_data>treshold){
            air_pressure_vertices.insert(make_pair(*vertex_iterator,air_pressure_vertices_data));
        }

    }
    return air_pressure_vertices;
}
*/

