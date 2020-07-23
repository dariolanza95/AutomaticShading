#ifndef MATERIALCLASSIFIER_H
#define MATERIALCLASSIFIER_H
#include "aclassifier.h"
#include "AShader.h"
#include "ShaderWrapper.h"
#include "materialshader.h"
class MaterialClassifier : public  AClassifier
{
public:
    MaterialClassifier(MyMesh &mesh);
    map<MyMesh::VertexHandle,std::shared_ptr<AShader>> ClassifyVertices();
    std::shared_ptr<AShader> GetShader();
};

#endif // MATERIALCLASSIFIER_H
