#ifndef POINTCLOUDWRITER_H
#define POINTCLOUDWRITER_H
#include <pointcloud.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "ShaderWrapper.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#ifdef Success
  #undef Success
#endif
#include "LICMap.h"
#include "utils.h"
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;


class PointCloudWriter
{

    int _num_shader_parameters;
    PtcPointCloud _output_file;
    std::string _output_file_name;
    MyMesh _mesh;
    AShader* _shader;
    bool _writing_a_shader_mask;
    int _subdiv_levels;
    std::vector<float > allocated_data;
    std::string _output_path;
    char* CreateMaskFile(AShader* shader,std::vector<char*>& var_types,std::vector<char*>& var_names,int& num_variables);

public:
    void Write();
    void Read();
    PointCloudWriter(MyMesh mesh, AShader* shader, int _subdiv_levels, std::string output_path,bool mask);
};

#endif // POINTCLOUDWRITER_H
