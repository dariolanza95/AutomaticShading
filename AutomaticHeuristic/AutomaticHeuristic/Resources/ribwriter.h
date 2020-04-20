#ifndef RIBWRITER_H
#define RIBWRITER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include "ribnode.h"
#include "Graphics/Camera.h"
#include "ShaderWrapper.h"
#include <glm/gtc/type_ptr.hpp>
#include "VertexEditTag.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

using namespace std;

class RIBWriter
{
    ofstream _rib_file ;
    MyMesh _mesh;
    Camera _cam;
    vector<VertexEditTag> _vertex_edit_tag;
    void WriteVertexEditTag();
    void RotateAlongX(glm::mat4x4 & mat,float angle);
    void  WriteTransformationMatrix();
    void  WriteNumOfVerticesInEachFace();
    void  WriteIndexBuffer();
    void  WriteSimulationData();
    void  InstanceLandscape();
    void WriteVertices();
    void CopyInitialPart(ifstream &myfile);
    void CopyFinalPart(ifstream &myfile);
    void SetUpMaterials();
public:
    RIBWriter(MyMesh mesh,string out_name_file,Camera _cam,vector<VertexEditTag> list_of_vertices_change);
    void Write();
    void WriteArchive();
    void InsertVertices();
    void InsertSimulationData();
    void MixNode(RIBNode node1,RIBNode node2,RIBNode node3);
    void MixNode();
    void Shader();
};

#endif // RIBWRITER_H
