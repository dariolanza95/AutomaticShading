#ifndef FEATURESFINDER_H
#define FEATURESFINDER_H
#include "aclassifier.h"
#include "screeclassifier.h"
#include "riverclassifier.h"
#include "flowclassifier.h"
#include "materialclassifier.h"
#include "VertexEditTag.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include "sedimentationclassifier.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <math.h>
class FeaturesFinder
{

    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud;
    std::vector<ShadersWrapper*> list_of_shaders_wrappers;
    MyMesh& _mesh;
  //  void UpdateSimulationData(map<MyMesh::VertexHandle,AShader*> selected_vertices);
    void UpdateSimulationData(std::vector<glm::vec3> list_of_points, std::vector<AShader*> list_of_data,float density);

    void InitializerSimulationData();
    vector<VertexEditTag> _vertex_edit_tags;
public:
    const std::vector<ShadersWrapper*>  getListOfShadersWrapper();
    pcl::PointCloud<pcl::PointXYZL>::Ptr  const getPointClouds();
    FeaturesFinder(MyMesh& mesh);
    vector<VertexEditTag> GetVertexEditTags();
    MyMesh& Find(std::vector<AShader* > &list_of_used_shaders);
};

#endif // FEATURESFINDER_H
