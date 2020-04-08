#ifndef PCL_H
#define PCL_H
#include "ShaderParameters.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <pcl/cloud_iterator.h>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <./../Noise/FastNoise/FastNoise.h>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>



typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;




class LICMap
{
    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Outputcloud;
public:
    LICMap(MyMesh mesh);
    float GetPoint(float point[3]);
    void LIC(float box_length,float frequency,float step_size,MyMesh mesh);

};

#endif // PCL_H
