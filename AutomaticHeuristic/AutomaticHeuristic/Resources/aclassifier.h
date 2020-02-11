#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>
#include "ShaderParameters.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;

class AClassifier
{
protected:
    static int _id;
public:
    virtual map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices() = 0;
    AClassifier();
};



#endif // ACLASSIFIER_H
