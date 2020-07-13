#ifndef POINTCLOUDWRITER_H
#define POINTCLOUDWRITER_H
#include <pointcloud.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "apointcloudwriter.h"
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
#include "mydefwrapper.h"
//typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
#include "featuresfinder.h"

class RIBPointCloudWriter: public APointCloudWriter
{
    //FeaturesFinder _features_finder;
    int _num_shader_parameters;
    PtcPointCloud _output_file;
    std::string _output_file_name;
    MyMesh _mesh;
    std::shared_ptr<AShader> _shader;
    bool _writing_a_shader_mask;
    int _subdiv_levels;
    std::vector<float > allocated_data;
    std::string _output_path;

    char* CreateMaskFile(std::shared_ptr<AShader> shader,std::vector<char*>& var_types,std::vector<char*>& var_names,int& num_variables);
    void WritePointCloud(std::shared_ptr<AShader> shader);
public:

    void Read();
    //RIBPointCloudWriter(MyMesh mesh, std::shared_ptr<AShader> shader);
    RIBPointCloudWriter(MyMesh mesh,
                        std::string _output_path,
                        std::vector<shared_ptr<AShader> > list_of_used_shaders,
                        std::vector<shared_ptr<ShadersWrapper>> list_of_shaders_wrapper,
                        std::vector<pcl::PointXYZL,Eigen::aligned_allocator<pcl::PointXYZL>>  list_of_used_points);
};

#endif // POINTCLOUDWRITER_H
