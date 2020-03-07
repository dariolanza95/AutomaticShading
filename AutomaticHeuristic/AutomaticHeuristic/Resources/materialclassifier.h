#ifndef MATERIALCLASSIFIER_H
#define MATERIALCLASSIFIER_H
#include "aclassifier.h"
#include "ShaderParameters.h"
class MaterialClassifier : public  AClassifier
{
public:
    MaterialClassifier(MyMesh mesh);
    map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices();

};

#endif // MATERIALCLASSIFIER_H
