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
    int _shader_parameter_size;
    MyMesh _mesh;
    PropertyManager<typename HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    template <typename T,typename FuncType>
    map<MyMesh::VertexHandle,T> BFS(int max_depth,map<MyMesh::VertexHandle,T> frontier_map,FuncType pred);
    map<MyMesh::VertexHandle,float> selectFrontier(map<MyMesh::VertexHandle,float> river_vertices);
    map<MyMesh::VertexHandle,ShaderParameters*> SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries);
    map<MyMesh::VertexHandle,float> SelectRiverVertices();
    vector<map<MyMesh::VertexHandle, ShaderParameters *>>DivideInGroups(map<MyMesh::VertexHandle,ShaderParameters*>& points_to_be_grouped);
    vector<map<MyMesh::VertexHandle, ShaderParameters *>> FindLocalMinimumValue(vector<map<MyMesh::VertexHandle,ShaderParameters*>> vector_of_groups);
    map<MyMesh::VertexHandle,ShaderParameters*> CollectGroups(vector<map<MyMesh::VertexHandle,ShaderParameters*>> groups);
    void FindMeshExtremes();
public:
    RiverClassifier(MyMesh mesh,float slope,float treshold,float border_width,float max_height,float min_height);
    map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices();
    friend class RiverClassifierTester;
};

#endif // RIVERCLASSIFIER_H
