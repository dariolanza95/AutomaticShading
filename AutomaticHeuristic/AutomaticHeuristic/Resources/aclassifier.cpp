#include "aclassifier.h"

int AClassifier::_id = 0;

AClassifier::AClassifier(MyMesh mesh) : _mesh(mesh)
{
    _id++;
    //simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
}
int AClassifier::GetId(){return _id;}
AClassifier::~AClassifier(){}
std::shared_ptr<AShader> AClassifier::GetShader(){return _shader;}
