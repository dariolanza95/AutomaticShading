#ifndef FLOWCLASSIFIER_H
#define FLOWCLASSIFIER_H
#include "aclassifier.h"
#include "VertexEditTag.h"
#include "simulationdata.h"
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
#include "flowshader.h"
#include <./../Noise/FastNoise/FastNoise.h>
#include "subdividerandinterpolator.h"
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RulesT.hh>
#include <pointcloud.h>
#include <unordered_set>
#include <list>
#include <random>
#include "myrules.h"
#include <hash_set>
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/kdtree/kdtree_flann.h>
class FlowClassifier : public AClassifier
{
    VertexEditTag _vertex_edit_tag;
    int _shader_parameter_size;
    map<MyMesh::VertexHandle,AShader*> LIC(map<MyMesh::VertexHandle,FlowShader*>const map,float scale,glm::vec3 min_bb,glm::vec3 max_bb);
    template <typename T,typename FuncType>
    map<MyMesh::VertexHandle,T> BFS(int max_depth,map<MyMesh::VertexHandle,T> frontier_map,FuncType pred);

    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud;
    pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_input;
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_input;

    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , FlowShader*>::type, MyMesh> flow_shader_temp_propery;
    void ContrastEnhancement(map<MyMesh::VertexHandle,AShader*>& map, std::vector<int> Pdf, int z);
    map<MyMesh::VertexHandle,FlowShader*> ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    void selectFrontier(map<MyMesh::VertexHandle,FlowShader*>& selected_vertices);

    //void ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    map<MyMesh::VertexHandle,glm::vec3> selectFlowVertices(glm::vec3& min_bb,glm::vec3& max_bb);
    void TemporaryUpdate(map<MyMesh::VertexHandle,FlowShader*> selected_vertices);

    //map<MyMesh::VertexHandle,ShaderParameters*> DebugFunction(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices);
    //void LIC2(float box_length,float frequency,float step_size,MyMesh mesh);
    //void RefineShaderParameters(MyMesh::VertexHandle vertex,ShaderParameters *shader_parameters);
public:
    VertexEditTag GetVertexEditTag();
    FlowClassifier(MyMesh& mesh);
    AShader* GetShader();
    map<MyMesh::VertexHandle,AShader*> ClassifyVertices();

};

#endif // FLOWCLASSIFIER_H
