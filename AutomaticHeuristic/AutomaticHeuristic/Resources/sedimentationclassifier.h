#ifndef SEDIMENTATIONCLASSIFIER_H
#define SEDIMENTATIONCLASSIFIER_H


#include "aclassifier.h"
#include <glm/vec3.hpp>
#include <glm/glm.hpp>

#include "flowshader.h"
#include "sedimentationshader.h"
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <hash_set>
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "subdividerandinterpolator.h"
#include "AShader.h"
#include "sedimentationdata.h"

class SedimentationClassifier : public AClassifier
{
    SimulationDataMap simulation_data_map;
    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_input;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_input;
    void AverageOutputData(float treshold);
    void AssignSedimentationParameters(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    void AssignSedimentationParameters2(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
void AverageData();

    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , std::shared_ptr<SimulationData>>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle, sedimentationData> SelectSedimentationPoints();
    std::vector<std::shared_ptr<AShader>> list_of_shaders;
    std::vector<glm::vec3> list_of_sedimentation_points;
    set<MyMesh::FaceHandle> GetSetOfFaces(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    void CreatePointCloud();
    void ComputeSedimentationParametersForVertex(glm::vec3 actual_point,sedimentationData& sedimenation_data);
public:
     ~SedimentationClassifier();
    SedimentationClassifier(MyMesh mesh,SimulationDataMap simulation_data_map);
     std::shared_ptr<AShader> GetShader();
    void ClassifyVertices(std::vector<glm::vec3>& list_of_points,
                                                   std::vector<std::shared_ptr<AShader>>& list_of_data,
                                                   float& details);
};
#endif // SEDIMENTATIONCLASSIFIER_H
