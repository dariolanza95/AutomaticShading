#ifndef SEDIMENTATIONCLASSIFIER_H
#define SEDIMENTATIONCLASSIFIER_H


#include "aclassifier.h"
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <boost/numeric/ublas/matrix.hpp>
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
#include <Eigen/Core>
#include <iostream>
#include "External/MathtoolBox/rbf-interpolation.hpp"
#include <random>
#include <vector>
#include "External/tps/ludecomposition.h"

enum class SedimentationClassifierAlgorithms{NP,NPTS,RBF};

class SedimentationClassifier : public AClassifier
{
    int subdiv_levels;
    bool useMockUp;
    void SelectTopMostVertices(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    SimulationDataMap simulation_data_map;
    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_input;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_input;
    SedimentationClassifierAlgorithms algorithm;
    void AverageOutputData(float treshold, int K);
    float CalculateTPS(std::vector<glm::vec3> control_points);
    void TPSFirstApprach(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    std::vector<int> SelectNeighbours(glm::vec3 actual_point);
    map<MyMesh::VertexHandle,sedimentationData> SelectPointsForAdaptiveSubdivision(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    void ComputeMockUpData(glm::vec3 actual_point,float size,int num_materials);
    void ComputeMockUpData_2(glm::vec3 actual_point,float size,int num_materials);

    void NP();
    void NPTS(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    void RBF(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);

    //void AssignSedimentationParameters3(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    //float RBFInterp(glm::vec3 actual_point,std::vector<glm::vec3> selected_points,std::vector<float> list_of_materials);
    void AverageData();
    double RBFInterp(glm::vec3 actual_point, std::vector<glm::vec3> selected_points);
    void AverageInputData(float treshold,float detail_scale,int K);
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , std::shared_ptr<SimulationData>>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle, sedimentationData> SelectSedimentationPoints();
    std::vector<std::shared_ptr<AShader>> list_of_shaders;
    std::vector<glm::vec3> list_of_sedimentation_points;
    set<MyMesh::FaceHandle> GetSetOfFaces(map<MyMesh::VertexHandle,sedimentationData> selected_vertices);
    void CreatePointCloud();
    void ComputeSedimentationParametersForVertex(glm::vec3 actual_point,sedimentationData& sedimenation_data);
public:

     ~SedimentationClassifier();
    SedimentationClassifier(MyMesh mesh,
                            SimulationDataMap simulation_data_map,
                            int subdivision_levels = 1 ,
                            SedimentationClassifierAlgorithms algorithm = SedimentationClassifierAlgorithms::NPTS,
                            bool useMockUp = false);
     std::shared_ptr<AShader> GetShader();
     void ClassifyVertices(std::vector<glm::vec3>& list_of_points,
                      std::vector<std::shared_ptr<AShader>>& list_of_data,
                      float& details,
                      const pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud,
                      const std::vector<std::shared_ptr<ShadersWrapper>>   list_of_shaders_wrappers);
    //void ClassifyVertices(std::vector<glm::vec3>& list_of_points,
    //                                               std::vector<std::shared_ptr<AShader>>& list_of_data,
    //                                               float& details);
};
#endif // SEDIMENTATIONCLASSIFIER_H
