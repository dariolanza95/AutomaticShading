#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;

class AClassifier
{
public:
    virtual map<MyMesh::FaceHandle,float> ClassifyVertices() = 0;
};



#endif // ACLASSIFIER_H
