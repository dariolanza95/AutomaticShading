#ifndef PCL_H
#define PCL_H
#include "ShaderParameters.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>


typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;




class LICMap
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

public:
    LICMap(MyMesh mesh);
    pcl::PointXYZRGB GetPoint(glm::vec3 point);
};

#endif // PCL_H
