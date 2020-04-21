#ifndef FEATURESFINDER_H
#define FEATURESFINDER_H
#include "aclassifier.h"
#include "screeclassifier.h"
#include "riverclassifier.h"
#include "flowclassifier.h"
#include "materialclassifier.h"
#include "VertexEditTag.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

class FeaturesFinder
{
    MyMesh _mesh;
    void UpdateSimulationData(map<MyMesh::VertexHandle,AShader*> selected_vertices);
    void InitializerSimulationData();
    vector<VertexEditTag> _vertex_edit_tags;

public:
    FeaturesFinder(MyMesh mesh);
    vector<VertexEditTag> GetVertexEditTags();
    MyMesh Find(std::vector<AShader* > &list_of_used_shaders);
};

#endif // FEATURESFINDER_H
