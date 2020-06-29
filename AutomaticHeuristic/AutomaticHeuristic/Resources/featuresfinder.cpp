#include "featuresfinder.h"

FeaturesFinder::FeaturesFinder(MyMesh mesh, SimulationDataMap simulation_data_map): _mesh(mesh),
    simulation_data_map(simulation_data_map){
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZL>);
point_cloud = cloud1;
    point_cloud->width = 1;
    point_cloud->height = 1;
}


pcl::PointCloud<pcl::PointXYZL>::Ptr  const FeaturesFinder::getPointClouds(){
    return point_cloud ;
}

std::vector<glm::vec3> FeaturesFinder::getList_of_Normals(){
    return list_of_normals;
}

std::vector<std::shared_ptr<ShadersWrapper>> FeaturesFinder::getListOfShadersWrapper(){
    return list_of_shaders_wrappers;
}


FeaturesFinder::~FeaturesFinder(){}
void  FeaturesFinder::Find(std::vector<std::shared_ptr<AShader>>& list_of_used_shaders)
{



    //angle of repose is usually between 33-37 degreee depending on the rock type
    float angle = 10;
    float treshold = 3;
     map<MyMesh::VertexHandle,std::shared_ptr<AShader>> selected_faces;
     InitializerSimulationData();



     /*AClassifier *sc = new ScreeClassifier(_mesh,angle,treshold);
     selected_faces = sc->ClassifyVertices();*/
     //UpdateSimulationData(selected_faces);
     selected_faces.clear();
     //AClassifier *rc = new RiverClassifier(_mesh,75,20,5,34,-35);
     //selected_faces = rc->ClassifyVertices();
     //UpdateSimulationData(selected_faces);
     std::vector<glm::vec3> temp_list_of_points;
     std::vector<std::shared_ptr<AShader>> temp_list_of_shader;
     float details;
/*
     AClassifier *fc = new FlowClassifier(_mesh,simulation_data_map);
      fc->ClassifyVertices(temp_list_of_points,temp_list_of_shader,details);
     //if(selected_faces.size()>0)
     {
         UpdateSimulationData(temp_list_of_points,temp_list_of_shader,details);
         std::shared_ptr<AShader> shad = fc->GetShader();
         list_of_used_shaders.push_back(shad);
         selected_faces.clear();
     }
    delete fc;*/

      AClassifier *sc = new SedimentationClassifier(_mesh,simulation_data_map);
      sc->ClassifyVertices(temp_list_of_points,temp_list_of_shader,details);
  //  std::shared_ptr<SedimentationClassifier>  sc1 = std::static_pointer_cast<SedimentationClassifier> (sc);
    SedimentationClassifier* sc1 = (SedimentationClassifier*) sc;
      list_of_normals = sc1->list_of_normals;
      UpdateSimulationData(temp_list_of_points,temp_list_of_shader,details);
      std::shared_ptr<AShader> shad = sc->GetShader();
     list_of_used_shaders.push_back(shad);
     selected_faces.clear();
     delete sc;
     //delete sc1;


     /*UpdateSimulationData(selected_faces);
     AClassifier *mt = new MaterialClassifier(_mesh);

     selected_faces = mt->ClassifyVertices();

     if(selected_faces.size()>0)
     {
         UpdateSimulationData(selected_faces);
         selected_faces.clear();
         //std::shared_ptr<AShader> shad = mt->GetShader();
        //list_of_used_shaders.push_back(shad);
     }
*/
     //RiverClassifierTester rct;
     //rct.Test();
  //  return _mesh;
}


vector<VertexEditTag> FeaturesFinder::GetVertexEditTags()
{
    return _vertex_edit_tags;
}

void FeaturesFinder::UpdateSimulationData(std::vector<glm::vec3> list_of_points, std::vector<std::shared_ptr<AShader>> list_of_data,float density){

    int previous_width = point_cloud->width;
    if(list_of_points.size() != list_of_data.size()){
        std::cout<<"lists size dont matches"<<std::endl;
        exit(1);
    }

    if(previous_width==1)   {

        //point_cloud->resize(point_cloud->width);
        for(size_t i = 0;i<list_of_points.size();i++){
            glm::vec3 actual_point= list_of_points[i];
            pcl::PointXYZL newPoint;
            newPoint.x = actual_point[0];
            newPoint.y = actual_point[1];
            newPoint.z = actual_point[2];
            point_cloud->push_back(newPoint);
            std::shared_ptr<ShadersWrapper> sw(std::shared_ptr<ShadersWrapper>(new ShadersWrapper()));
            sw->AddShaderParameters(list_of_data[i]);
            list_of_shaders_wrappers.push_back(sw);
        }
        point_cloud->width = list_of_points.size();
        if(point_cloud->points.size()>0)
            kdtree.setInputCloud(point_cloud);
    }else   {

        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZL>);
        cloud1->points.resize (list_of_points.size());


        for(size_t i = 0;i<list_of_points.size();i++){
            std::cout<<" i "<<i<<" over "<<list_of_points.size()<<std::endl;
            glm::vec3 actual_point= list_of_points[i];
            pcl::PointXYZL searchPoint;
            searchPoint.x = actual_point[0];
            searchPoint.y = actual_point[1];
            searchPoint.z = actual_point[2];
            int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if(point_cloud->points.size()>0)
            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                if(sqrtf(pointNKNSquaredDistance[0])<= density){
                    int idx = pointIdxNKNSearch[0];
                    list_of_shaders_wrappers[idx]->AddShaderParameters(list_of_data[i]);
                }
                else{
                    cloud1->points.push_back(searchPoint);
                    std::shared_ptr<ShadersWrapper> sw(std::shared_ptr<ShadersWrapper>(new ShadersWrapper()));
                    sw->AddShaderParameters(list_of_data[i]);
                    list_of_shaders_wrappers.push_back(sw);
                }
            }
        }
        for(size_t i = 0;i<list_of_points.size();i++){
            pcl::PointXYZL searchPoint =  cloud1->points[i];
            point_cloud->points.push_back(searchPoint);
        }

        kdtree.setInputCloud(point_cloud);

    }
}


/*
void FeaturesFinder::UpdateSimulationData(map<MyMesh::VertexHandle,std::shared_ptr<AShader>> selected_vertices)
{
    std::cout<<"UpdateSimData"<<std::endl;
    auto shader_parameters_property = getOrMakeProperty<VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");
    for (auto const& x : selected_vertices)    {
        MyMesh::VertexHandle vertex_handle = x.first;
        if(vertex_handle.idx() == 90010){
            std::cout<<"breakpoint"<<std::endl;
        }
        std::shared_ptr<AShader> shader = x.second;
        ShadersWrapper* shaders_wrapper = shader_parameters_property[vertex_handle];
        shaders_wrapper->AddShaderParameters(shader);
        //        shader_parameters_property[vertex_handle] = shader_parameter;

    }
}
*/
void FeaturesFinder::InitializerSimulationData()
{
//auto shader_parameters_data_wrapper= getOrMakeProperty<VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");
//
    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    for(vertex_iterator = _mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        if(simulation_data_map.count(*vertex_iterator)<=0){

            simulation_data_map.insert(make_pair(*vertex_iterator,std::shared_ptr<SimulationData>(new SimulationData())));
//                    [*vertex_iterator] = new SimulationData();
        }
//        ShadersWrapper* shaders_wrapper= new ShadersWrapper();
//        shader_parameters_data_wrapper[*vertex_iterator] = shaders_wrapper;
    }
}
