#ifndef APOINTCLOUDWRITER_H
#define APOINTCLOUDWRITER_H
#include "AShader.h"
#include <memory>
#include "ShaderWrapper.h"
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/point_types.h>
class APointCloudWriter
{
    virtual void WritePointCloud(std::shared_ptr<AShader> shader) = 0;
    bool write_pointcloud_mask;
protected:
    std::vector<std::shared_ptr<AShader>> list_of_used_shaders;
    std::vector<std::shared_ptr<ShadersWrapper>> list_of_shaders_wrapper;
    std::vector<pcl::PointXYZL,Eigen::aligned_allocator<pcl::PointXYZL>>  list_of_used_points;
public:
  APointCloudWriter(std::vector<std::shared_ptr<AShader> > list_of_used_shaders,
                    std::vector<std::shared_ptr<ShadersWrapper>> list_of_shaders_wrapper,
                    std::vector<pcl::PointXYZL,Eigen::aligned_allocator<pcl::PointXYZL>>  list_of_used_points);
  void WritePointClouds();
  void getCloudPathNameInterface(std::string& path,std::shared_ptr<AShader> shader);
  void getSerializedTypesInterface(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables,std::shared_ptr<AShader> shader);
  void getSerializedDataInterface(std::vector<float> &data,std::shared_ptr<AShader> shader);

};

#endif // APOINTCLOUDWRITER_H
