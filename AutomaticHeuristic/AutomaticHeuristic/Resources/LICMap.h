#ifndef PCL_H
#define PCL_H
#include "ShaderWrapper.h"
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
#include <External/FastNoise/FastNoise.h>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>

#include "subdividerandinterpolator.h"
#include "./mydefwrapper.h"





class LICMap
{
    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Outputcloud;
    void ReIterationLIC(float box_length,float frequency,float step_size,MyMesh mesh);
    MyMesh _mesh;
public:
    LICMap(MyMesh mesh,int subdivision_levels);
    float GetPoint(float point[3]);
    
    void LIC2(float box_length,float frequency,float step_size,MyMesh mesh);
    //void LIC2(float box_length,float frequency,float step_size,MyMesh _mesh);
    int _subdiv_levels;
};

#endif // PCL_H
