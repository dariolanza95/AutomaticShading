#ifndef RIBWRITER_H
#define RIBWRITER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include "ribshadernode.h"
#include "Graphics/Camera.h"
#include "ShaderWrapper.h"
#include <glm/gtc/type_ptr.hpp>
#include "VertexEditTag.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "ribmasknode.h"
#include "ribconstant.h"
#include "ribmixnode.h"
#include "bxdfnode.h"
#include "ribaddnode.h"
#include "riblight.h"
#include "displnode.h"
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

using namespace std;

class RIBWriter
{
    ofstream _rib_file ;
    MyMesh _mesh;
    Camera _cam;
    BXDFNode* _bxdf_node;
    DisplNode* _displ_node;
    vector<VertexEditTag> _vertex_edit_tag;
    vector<AShader*> _list_of_used_shaders;

    string _plugins_path;
    string _shaders_path;
    string _procedural_path;
    string _output_path;
    string _output_name_image;
    void WriteVertexEditTag();
    void RotateAlongX(glm::mat4x4 & mat,float angle);
    string WriteTransformationMatrix();
    void  WriteNumOfVerticesInEachFace();
    void  WriteIndexBuffer();
    void  WriteSimulationData();
    void  InstanceLandscape();
    void WriteVertices();
    void WriteDisplacementLogic();
    void CopyInitialPart(ifstream &myfile);
    void CopyFinalPart(ifstream &myfile);
    void SetUpMaterials();
    void WriteShadersList();
    void WriteBXDF(RIBNode* node);
    void WriteInialization();
public:
    RIBWriter(MyMesh mesh,string out_name_file,string shaders_path,string plugins_path,string output_name_image,Camera _cam,vector<AShader*> list_of_used_shaders);
    void Write();
    void WriteArchive();
    void InsertVertices();
    void InsertSimulationData();
//    RIBShaderNode MixNode(string name,RIBShaderNode mixNode,RIBShaderNode color1,RIBShaderNode color2);

    //void MixNode(RIBNode node1, RIBNode node2, RIBNode node3);
    void Shader();
};

#endif // RIBWRITER_H
