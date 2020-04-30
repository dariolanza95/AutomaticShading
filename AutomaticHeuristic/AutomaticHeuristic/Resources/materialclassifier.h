#ifndef MATERIALCLASSIFIER_H
#define MATERIALCLASSIFIER_H
#include "aclassifier.h"
#include "Ashader.h"
#include "ShaderWrapper.h"
#include "materialshader.h"
class MaterialClassifier : public  AClassifier
{
public:
    MaterialClassifier(MyMesh &mesh);
    map<MyMesh::VertexHandle,AShader*> ClassifyVertices();
    AShader* GetShader();
};

#endif // MATERIALCLASSIFIER_H
