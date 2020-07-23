#ifndef POINTCLOUDWRITERTESTER_H
#define POINTCLOUDWRITERTESTER_H
#include <pointcloud.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "ShaderWrapper.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

class PointCloudWriterTester
{
    std::string _file_name;
public:
    PointCloudWriterTester(std::string input_filename);
};

#endif // POINTCLOUDWRITERTESTER_H
