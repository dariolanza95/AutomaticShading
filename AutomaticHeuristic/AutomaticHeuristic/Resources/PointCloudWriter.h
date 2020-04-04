#ifndef POINTCLOUDWRITER_H
#define POINTCLOUDWRITER_H
#include <pointcloud.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "ShaderParameters.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#ifdef Success
  #undef Success
#endif
#include "LICMap.h"
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;


class PointCloudWriter
{
    int _num_shader_parameters;
    PtcPointCloud _output_file;
    std::string _output_file_name;
    LICMap _licmap;
    MyMesh _mesh;
public:
    void Init();
    void Write();
    void Read();
    PointCloudWriter(MyMesh mesh,std::string input_file_name,int num_shader_parameters,LICMap licmap);
};

#endif // POINTCLOUDWRITER_H
