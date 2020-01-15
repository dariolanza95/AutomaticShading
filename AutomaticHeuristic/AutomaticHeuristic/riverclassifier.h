#ifndef RIVERCLASSIFIER_H
#define RIVERCLASSIFIER_H
#include "aclassifier.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
using namespace OpenMesh;
class RiverClassifier: public AClassifier
{
    float _slope;
    float _treshold;
    float _border_width;
    MyMesh _mesh;
    PropertyManager<typename HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle,float> BFS(int depth,map<MyMesh::VertexHandle,float> frontier_map);
    map<MyMesh::VertexHandle,float> selectFrontier(map<MyMesh::VertexHandle,float> river_vertices);
    map<MyMesh::FaceHandle,float> SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries);
public:
    RiverClassifier(MyMesh mesh,float slope,float treshold,float border_width);
    map<MyMesh::FaceHandle,float> ClassifyVertices();
    friend class RiverClassifierTester;
};

#endif // RIVERCLASSIFIER_H
