#ifndef RIVERCLASSIFIER_H
#define RIVERCLASSIFIER_H
#include "aclassifier.h"
#include "simulationdata.h"
#include <math.h>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
using namespace OpenMesh;
class RiverClassifier: public AClassifier
{
    float _slope;
    float _treshold;
    float _border_width;
    float _max_height;
    float _min_height;
    MyMesh _mesh;
    PropertyManager<typename HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle,float> BFS(int depth,map<MyMesh::VertexHandle,float> frontier_map);
    map<MyMesh::VertexHandle,float> selectFrontier(map<MyMesh::VertexHandle,float> river_vertices);
    map<MyMesh::VertexHandle,ShaderParameters*> SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries);
    map<MyMesh::VertexHandle,float> SelectRiverVertices();
public:
    RiverClassifier(MyMesh mesh,float slope,float treshold,float border_width,float max_height,float min_height);
    map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices();
    friend class RiverClassifierTester;
};

#endif // RIVERCLASSIFIER_H
