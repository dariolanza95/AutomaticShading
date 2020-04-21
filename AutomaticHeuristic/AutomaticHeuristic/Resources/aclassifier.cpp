#include "aclassifier.h"

int AClassifier::_id = 0;

AClassifier::AClassifier(MyMesh mesh) : _mesh(mesh)
{
    _id++;
    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
}
int AClassifier::GetId(){return _id;}
AShader* AClassifier::GetShader(){return _shader;}
//template <typename T,typename FuncType>
//map<MyMesh::VertexHandle,T> BFS(int max_depth,map<MyMesh::VertexHandle,T> frontier_map,FuncType pred);
