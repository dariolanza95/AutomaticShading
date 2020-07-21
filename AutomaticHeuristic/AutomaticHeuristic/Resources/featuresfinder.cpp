#include "featuresfinder.h"

ClassificationAndDataComputationModule::ClassificationAndDataComputationModule(MyMesh mesh, SimulationDataMap simulation_data_map): _mesh(mesh),
    simulation_data_map(simulation_data_map){
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZL>);
point_cloud = cloud1;
    point_cloud->width = 1;
    point_cloud->height = 1;
}


pcl::PointCloud<pcl::PointXYZL>::Ptr  const ClassificationAndDataComputationModule::getPointClouds(){
    return point_cloud ;
}

std::vector<std::shared_ptr<ShadersWrapper>> ClassificationAndDataComputationModule::getListOfShadersWrapper(){
    return list_of_shaders_wrappers;
}
std::vector<std::shared_ptr<AShader>> ClassificationAndDataComputationModule::getListOfUsedShaders(){
    return list_of_used_shaders;
}

ClassificationAndDataComputationModule::~ClassificationAndDataComputationModule(){}
void  ClassificationAndDataComputationModule::Find()
{
    std::vector<glm::vec3> list_of_points;
    std::vector<std::shared_ptr<AShader>> list_of_appearance_data;
    float details = 0.1;
    std::vector<std::shared_ptr<AClassifier>> list_of_classifiers;

    list_of_classifiers.push_back(std::shared_ptr<AClassifier>(new FlowClassifier(_mesh,simulation_data_map,1)));
    list_of_classifiers.push_back(std::shared_ptr<AClassifier>(new SedimentationClassifier(_mesh,simulation_data_map,1)));

    int iterations = 1;

    InitializerSimulationData();

    std::chrono::high_resolution_clock clock;
    std::chrono::high_resolution_clock::time_point currentTime, newTime;
    std::chrono::high_resolution_clock::duration totalTime;
    currentTime = clock.now();
    list_of_points.clear();
    list_of_appearance_data.clear();
    /*for(std::shared_ptr<AClassifier> classifier : list_of_classifiers){
        classifier->ClassifyVertices(list_of_points,list_of_appearance_data,details);

        if(list_of_points.size() != list_of_appearance_data.size()){
            std::cout<<"The two output list don't match!";
        }
        else{
            if(list_of_points.size()>0)
            {
                UpdateSharedData(list_of_points,list_of_appearance_data,details);

                //Add the related AShader class to the list of used shaders
                std::shared_ptr<AShader> shad = classifier->GetShader();
                list_of_used_shaders.push_back(shad);

            }
        }
        list_of_points.clear();
        list_of_appearance_data.clear();
    }
    newTime = clock.now();
        totalTime = newTime - currentTime;
        double ms = std::chrono::duration_cast<std::chrono::milliseconds>(totalTime).count();
        double avg = ms / (double) iterations;
        std::cout<<"total time "<< ms << " ms, iterations "<<iterations<<" avg "<<avg<<std::endl;
*/
AClassifier *fc;
    //FOR testing purposes

  //  for(int i = 0;i<iterations;i++)
  //   {
         fc = new FlowClassifier(_mesh,simulation_data_map,1);
           fc->ClassifyVertices(list_of_points,list_of_appearance_data,details);
    //}

           if(list_of_points.size() != list_of_appearance_data.size()){
               std::cout<<"The two output list don't match!";
           }
           else{
               if(list_of_points.size()>0)
               {
                   UpdateSharedData(list_of_points,list_of_appearance_data,details);
                   std::shared_ptr<AShader> shad = fc->GetShader();
                   list_of_used_shaders.push_back(shad);
               }
           }
            list_of_points.clear();
           list_of_appearance_data.clear();


     AClassifier *sc = new SedimentationClassifier(_mesh,simulation_data_map);
      sc->ClassifyVertices(list_of_points,list_of_appearance_data,details);


      if(list_of_points.size() != list_of_appearance_data.size()){
          std::cout<<"The two output list don't match!";

      }else{
          if(list_of_points.size()>0)
          {
           UpdateSharedData(list_of_points,list_of_appearance_data,details);
           std::shared_ptr<AShader> shad = sc->GetShader();
          list_of_used_shaders.push_back(shad);
          }
      }
      newTime = clock.now();
          totalTime = newTime - currentTime;
          double mins = std::chrono::duration_cast<std::chrono::minutes>(totalTime).count();
          double avg = mins / (double) iterations;
          std::cout<<"total time "<< mins << " mins, iterations "<<iterations<<" avg "<<avg<<std::endl;
    // delete sc;*/
}


void ClassificationAndDataComputationModule::UpdateSharedData(std::vector<glm::vec3> list_of_points, std::vector<std::shared_ptr<AShader>> list_of_data,float density){

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

        int percentual_part = list_of_points.size()/100;
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZL>);
        //cloud1->points.resize (list_of_points.size());
        for(size_t i = 0;i<list_of_points.size();i++){

            if(i%percentual_part == 0)
                std::cout<< "Data transfer to shared kd-tree "<<i / percentual_part<< " %  completed"<< std::endl;

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
        for(size_t i = 0;i<cloud1->points.size();i++){
            pcl::PointXYZL searchPoint =  cloud1->points[i];
            point_cloud->points.push_back(searchPoint);
        }

        kdtree.setInputCloud(point_cloud);

    }
}


void ClassificationAndDataComputationModule::InitializerSimulationData()
{
//auto shader_parameters_data_wrapper= getOrMakeProperty<VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");
//
    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    for(vertex_iterator = _mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        if(simulation_data_map.count(*vertex_iterator)<=0){
            std::cout<<"Shouldnt happen"<<std::endl;
            simulation_data_map.insert(make_pair(*vertex_iterator,std::shared_ptr<SimulationData>(new SimulationData())));
        }

    }
}
