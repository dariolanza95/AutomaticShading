#include "sedimentationclassifier.h"



SedimentationClassifier::SedimentationClassifier(MyMesh& mesh) : AClassifier(mesh)
{_shader = new SedimentationShader(_id); }


void SedimentationClassifier::AssignSedimentationParameters(map<MyMesh::VertexHandle,sedimentationData> selected_vertices) {
    std::vector<AShader*>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++
    for(auto const entry : selected_vertices){
        pcl::PointXYZLNormal new_point;
        MyMesh::Point actual_point = _mesh.point(entry.first);

        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];

  pcl::PointXYZLNormal temp_point;
        int K = 1;//or 27
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            temporary_list_of_sedimentation_points.push_back(glm::vec3(actual_point[0],actual_point[1],actual_point[2]));
            SedimentationShader* shdr =(SedimentationShader*) list_of_shaders[pointIdxNKNSearch[0]];
            AShader* shad = new SedimentationShader(*shdr);
            temporary_list_of_shaders.push_back(shad);
            /*for(int b = 0 ;b<pointIdxNKNSearch.size();b++){
                temp_point =  cloud_input->points[ pointIdxNKNSearch[b]];
                new_point.normal_x += temp_point.normal_x;
                new_point.normal_y += temp_point.normal_y;
                new_point.normal_z += temp_point.normal_z;*/
            }


    }
    list_of_sedimentation_points = temporary_list_of_sedimentation_points;
    list_of_shaders = temporary_list_of_shaders;
}

map<MyMesh::VertexHandle,sedimentationData> SedimentationClassifier::SelectSedimentationPoints(){
    map<MyMesh::VertexHandle,sedimentationData> selected_vertices;
    MyMesh::VertexIter vertex_iterator = _mesh.vertices_begin();
    MyMesh::VertexIter vertex_iterator_end = _mesh.vertices_end();
    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");

    for(;vertex_iterator != vertex_iterator_end;++vertex_iterator){
        SimulationData* sd  = simulation_data_wrapper[*vertex_iterator];
           std::vector<float> sediment_history;
           sd->getData(SimulationDataEnum::sedimentation_history,sediment_history);
           glm::vec3 initial_sedimentation_point;
           glm::vec3 pp;
           sd->getData(SimulationDataEnum::initial_sedimentation_point,initial_sedimentation_point);
           sd->getData(SimulationDataEnum::actual_point,pp);
           if(sediment_history.size()>0 ){
               MyMesh::Point point = _mesh.point(*vertex_iterator);
               glm::vec3 actual_point (point[0],point[1],point[2]);
               sedimentationData sdtmp = sedimentationData(initial_sedimentation_point,sediment_history);
            sedimentationData& sd = sdtmp;

            selected_vertices.insert(std::make_pair(vertex_iterator,sd));
           }
    }

return selected_vertices;
}
/*
SedimentationClassifier::CreatePointCloud(){

}
*/
void SedimentationClassifier::ComputeSedimentationParametersForVertex(glm::vec3 actual_point,
                                                                      sedimentationData& sedimenation_data){
    if(sedimenation_data.sediment_history.size()==1)   {

        if(actual_point[2]- sedimenation_data.initial_position[2] > 0.01){

            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, 0));
            list_of_sedimentation_points.push_back(actual_point);


            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, sedimenation_data.sediment_history[0]));
            list_of_sedimentation_points.push_back(sedimenation_data.initial_position);
        }

    }else   {
        //float tmp = actual_point[0] ;
        //actual_point[0] = actual_point[1];
        //actual_point[1] = tmp;
        glm::vec3 initial_point = sedimenation_data.initial_position;
    //    float tmp = initial_point[0];
    //    initial_point[0] = initial_point[1];
    //    initial_point[1] = tmp;
        glm::vec3 distance_vector =   actual_point - initial_point;
        int num_divisions =    sedimenation_data.sediment_history.size() ;//- 2 + 1;
        float mod = sqrtf(dot(distance_vector,distance_vector));
        float step_length = mod / (float) num_divisions;
        glm::vec3 normalized_distance_vector = distance_vector/mod;
        for(uint i = 0 ;i<sedimenation_data.sediment_history.size()+1;i++){
            glm::vec3 new_sedimentation_point;
            new_sedimentation_point=  initial_point + normalized_distance_vector * step_length * (float )i ;
            if(i==0){
                list_of_shaders.push_back(new SedimentationShader(_id,1.0f,0.0f));
                            }else{

                list_of_shaders.push_back(new SedimentationShader(_id,1.0f, sedimenation_data.sediment_history[i-1]));
            }

            list_of_sedimentation_points.push_back(new_sedimentation_point);

//float temp2 =            new_sedimentation_point[0];
//new_sedimentation_point[0] = new_sedimentation_point[1];
//new_sedimentation_point [1] = temp2;
        }
    }
}


set<MyMesh::FaceHandle> SedimentationClassifier::GetSetOfFaces(map<MyMesh::VertexHandle,sedimentationData> selected_vertices){
    set<MyMesh::FaceHandle> selected_faces;
    for(auto const entry: selected_vertices){
        MyMesh::VertexFaceIter vertex_face_circulator;
        vertex_face_circulator = _mesh.vf_iter(entry.first);
        for( ;vertex_face_circulator.is_valid(); ++vertex_face_circulator){
            //if(_mesh.valence(vertex_face_circulator)==4 || j == 0)
                selected_faces.insert(vertex_face_circulator);
        }
    }
    return selected_faces;
}

void SedimentationClassifier::CreatePointCloud(){

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
      cloud_input = cloud1;

    cloud_input->width = list_of_sedimentation_points.size();//numpoints
    cloud_input->height = 1;
    cloud_input->points.resize (cloud_input->width * cloud_input->height);
    for(uint j = 0;j<list_of_sedimentation_points.size();j++){
    glm::vec3 const entry = list_of_sedimentation_points[j];
          cloud_input->points[j].x = entry[0];
          cloud_input->points[j].y = entry[1];
          cloud_input->points[j].z = entry[2];

        }
    kdtree_input.setInputCloud(cloud_input);
}

void SedimentationClassifier::ClassifyVertices(std::vector<glm::vec3>& list_of_points,
                                               std::vector<AShader*>& list_of_data,
                                               float& details){




    auto selected_vertices = SelectSedimentationPoints();
    for(auto entry : selected_vertices) {
        MyMesh::Point point = _mesh.point(entry.first);
        glm::vec3 actual_point (point[0],point[1],point[2]);
        ComputeSedimentationParametersForVertex(actual_point,entry.second);
        //delete entry.second;
    }
    CreatePointCloud();

     int _subdiv_levels = 2;
     for(int i=0;i<_subdiv_levels;i++){
         auto set_of_faces = GetSetOfFaces(selected_vertices);
         SubdividerAndInterpolator<MyMesh> catmull(set_of_faces);
             catmull.attach(_mesh);
             catmull(1);
             catmull.detach();
             selected_vertices = SelectSedimentationPoints();

    }

    AssignSedimentationParameters(selected_vertices);

    list_of_points = list_of_sedimentation_points;
    list_of_data = list_of_shaders;
    details = 0.5;
}


