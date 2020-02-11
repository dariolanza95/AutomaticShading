#ifndef FEATURESFINDER_H
#define FEATURESFINDER_H
#include "aclassifier.h"
#include "screeclassifier.h"
#include "riverclassifier.h"
#include "flowclassifier.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

class FeaturesFinder
{
    MyMesh _mesh;
    void UpdateSimulationData(map<MyMesh::VertexHandle,ShaderParameters*> selected_vertices);
    void InitializerSimulationData();

public:
    FeaturesFinder(MyMesh mesh);
    MyMesh Find();
};

#endif // FEATURESFINDER_H
