#include "sedimentationclassifier.h"



SedimentationClassifier::SedimentationClassifier(MyMesh mesh) : AClassifier(mesh)
{_shader = new SedimentationShader(_id); }



void SedimentationClassifier::AverageData(){
    std::vector<AShader*>  temporary_list_of_shaders(list_of_sedimentation_points.size());
    std::vector<glm::vec3> temporary_list_of_sedimentation_points(list_of_sedimentation_points.size());
    std::vector<float> new_list_of_materials;
    std::vector<glm::vec3> new_list_of_points;
   //  temporary_list_of_sedimentation_points = list_of_sedimentation_points;
   // temporary_list_of_shaders = list_of_shaders  ;
    CreatePointCloud();
    for(uint i = 0;i<list_of_sedimentation_points.size();i++){
        pcl::PointXYZLNormal new_point;
        glm::vec3 actual_point = list_of_sedimentation_points[i];
        std::cout<<i<<" over" << list_of_sedimentation_points.size()<<std::endl;
        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];
    int id = 0;
  pcl::PointXYZLNormal temp_point;
        int K = 30;//or 27
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            //SedimentationShader* tmp_sed = temporary_list_of_shaders[pointIdxNKNSearch[0]];
            std::vector<std::vector<float>> list_of_lists(pointIdxNKNSearch.size());
            for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                list_of_lists[j] =  ((SedimentationShader*) list_of_shaders[pointIdxNKNSearch[j]])->getListOfIntermediateSedimentationMaterials();
            }
            SedimentationShader* tmp_sed = (SedimentationShader*) list_of_shaders[pointIdxNKNSearch[0]];
            float tot_size = 0;
            for(uint j  = 0;j<list_of_lists.size();j++){
                tot_size += list_of_lists[j].size();
            }
            tot_size /= list_of_lists.size();
           //new_list_of_materials = tmp_sed->getListOfIntermediateSedimentationMaterials();
            new_list_of_points = tmp_sed->getListOfIntermediateSedimentationPoints();
           int new_size = roundf(tot_size);
            new_list_of_materials.resize(new_size);
            for(uint j = 0;j<new_size;j++){
                float total_val = 0;
                for(uint z = 0;z<list_of_lists.size();z++){
                    if(list_of_lists[z].size()>j)
                        total_val+=list_of_lists[z][j];
                }
                float val_res  = roundf(total_val/(float)list_of_lists.size());
new_list_of_materials[j] = val_res;
            }

            glm::vec3 initial_point = new_list_of_points[0];
            glm::vec3 actual_point = new_list_of_points.back();
            new_list_of_points.resize(new_size);
            glm::vec3 distance_vector =   actual_point - initial_point;
            int num_divisions = new_size -1 ;//- 2 + 1;
            float mod = sqrtf(dot(distance_vector,distance_vector));
            float step_length = mod / (float) num_divisions;
            glm::vec3 normalized_distance_vector = distance_vector/mod;
            glm::vec3 new_sedimentation_point;
            for(uint i = 0 ;i<new_size;i++){
                new_sedimentation_point=  initial_point + normalized_distance_vector * step_length * (float )i ;
                new_list_of_points.push_back(new_sedimentation_point);
         }


            temporary_list_of_shaders[i] = new SedimentationShader(_id,1.0f,new_list_of_points,new_list_of_materials) ;
            new_list_of_materials.clear();
            new_list_of_points.clear();
    }

}
    list_of_shaders = temporary_list_of_shaders;
}

void SedimentationClassifier::AssignSedimentationParameters2(map<MyMesh::VertexHandle,sedimentationData> selected_vertices)
{
    std::vector<AShader*>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++

    for(auto const entry : list_of_shaders){
       std::vector<float> temp_mat;
       std::vector<glm::vec3> temp_pts;

        SedimentationShader* sd = (SedimentationShader*) entry;
       temp_mat = sd->getListOfIntermediateSedimentationMaterials();
       temp_pts = sd->getListOfIntermediateSedimentationPoints();
       for(uint i = 0;i<temp_mat.size();i++){
       //    if(temp_pts[0][2] > -9.5){
       //        temporary_list_of_shaders.push_back(new SedimentationShader(_id,1.0f,0));
       //    }else{

               temporary_list_of_shaders.push_back(new SedimentationShader(_id,1.0f,temp_mat[i]));
         //  }
temporary_list_of_sedimentation_points.push_back(temp_pts[i]);
       }
    }
    /*for(auto entry : selected_vertices){
         MyMesh::Point p = _mesh.point(entry.first);
         glm::vec3 actual_point(p[0],p[1],p[2]);
         sedimentationData sd = entry.second;
         //int height = roundf(p[2]+10);
         int j = 1;
         glm::vec3 in = sd.initial_position;
         float step_length = p[2]-in[2];
         step_length /= roundf(p[2]-in[2]);//sd.sediment_history.size();
         //for(float i = 0;i<=height;i+=0.1){
         if(p[2]-in[2]>0){
             for(float i = in[2];i<p[2];i+=step_length){
                 //glm::vec3 tmp_point(p[0],p[1],i-10);
                 glm::vec3 tmp_point(p[0],p[1],i );

//                 SedimentationShader* sedshad = new SedimentationShader(_id,1.0f,j++);
                 SedimentationShader* sedshad = new SedimentationShader(_id,1.0f,sd.sediment_history[j++]);
                if(j>3)
                    j= 1;
       //          if(j>=sd.sediment_history.size())
       //              j = 0;
                 temporary_list_of_sedimentation_points.push_back(tmp_point);
                 temporary_list_of_shaders.push_back(sedshad);
             }
         }
         else{
             std::cout<<"jere"<<std::endl;
         }

    }*/

list_of_sedimentation_points= temporary_list_of_sedimentation_points;
list_of_shaders = temporary_list_of_shaders;
}

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
    int id = 0;
        pcl::PointXYZLNormal temp_point;
        int K = 10;//or 27
        std::vector<int> pointIdxNKNSearch(K);
      //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            temporary_list_of_sedimentation_points.push_back(glm::vec3(actual_point[0],actual_point[1],actual_point[2]));
            glm::vec3 pp(actual_point[0],actual_point[1],actual_point[2]);
            float system_energy = 0;
            SedimentationShader* shad2 = (SedimentationShader*)  list_of_shaders[pointIdxNKNSearch[0]];
            uint index = shad2->getClosestPointIndex(pp);
            uint min_index = 0;
            float min_energy = INFINITY;
            float dist ;
            for(uint j = 0;j<pointNKNSquaredDistance.size();j++){
                    total_dist+= (pointNKNSquaredDistance[j]);
            }
        float normalization = 0;
            for(uint j = 0;j<pointNKNSquaredDistance.size();j++){
                    pointNKNSquaredDistance[j] = 1- (pointNKNSquaredDistance[j]/total_dist);
                    normalization+= (pointNKNSquaredDistance[j]);
            }
total_dist = normalization;
            std::vector<float> list = shad2->getListOfIntermediateSedimentationMaterials();
            for(int i = index - 1 ; i <index+1;i++){
                if(i< 0 || i >= list.size()){
                    continue;
                }
                system_energy = 0;
                for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                    SedimentationShader* shad = (SedimentationShader*)  list_of_shaders[pointIdxNKNSearch[j]];

                    //float weight = 1/(K-1);
                   float weight = pointNKNSquaredDistance[j]/total_dist;

//                    if(j!=0){
//                    weight = pointNKNSquaredDistance[j]/total_dist;
//                    //    weight = ((total_dist- pointNKNSquaredDistance[j])/total_dist);
//                    }
                    //float weight = 1;//-1/pointNKNSquaredDistance[j];
                    system_energy += shad->utilityFunct(pp,list[i])*weight;
                    //ids_list[j] = shad->getClosestPointMatId(pp,dist);
                    //dists_list[j] = dist;
                    //if(dist == 0){
                    //    id = j;
                    //    break;
                    //}
//                    total_dist += 1/dist;
                    //    dist = std::max(0.0001f,dist);
                //    total_dist += 1/dist;

                    //total += shad->getClosestPointMatId(pp,dist);
                    //sedimentation_materials_list[j] =(SedimentationShader*) list_of_shaders[pointIdxNKNSearch[j]];
                }
                if(system_energy<min_energy){
                    min_energy = system_energy;
                    min_index = i;
                }
            }
            /*if(pointIdxNKNSearch.size()>=2){
                id = sedimentation_materials_list[0]->GetMaterialId( *sedimentation_materials_list[1],glm::vec3(actual_point[0],actual_point[1],actual_point[2]));

            }*/
//if(dist!=0){
//    //total_dist = 1/total_dist;
//    float min_dist = INFINITY;
//    for(uint j = 1;j<pointIdxNKNSearch.size();j++){
//        if(dists_list[j]< min_dist){
//            min_dist = dists_list[j];
//            total = ids_list[j];
//        }
//    }
//    id = (total);
//}
id = list[min_index];

            //id = total/(float )pointIdxNKNSearch.size();
            AShader* shad  = new SedimentationShader(_id,1.0f,/*roundf*/(id));
            /*     AShader* shad = new SedimentationShader(*shdr);*/
            temporary_list_of_shaders.push_back(shad);

            }

    }
    list_of_sedimentation_points = temporary_list_of_sedimentation_points;
    list_of_shaders = temporary_list_of_shaders;
}

map<MyMesh::VertexHandle,sedimentationData> SedimentationClassifier::SelectSedimentationPoints(){
    map<MyMesh::VertexHandle,sedimentationData> selected_vertices;
    MyMesh::VertexIter vertex_iterator = _mesh.vertices_begin();
    MyMesh::VertexIter vertex_iterator_end = _mesh.vertices_end();
   simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "simulation_data");

//    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "mazulanzas");
int i = 0;
    for(;vertex_iterator != vertex_iterator_end;++vertex_iterator){

        //SimulationData* sd ;
        std::shared_ptr<SimulationData> sd = simulation_data_wrapper[*vertex_iterator];
           std::vector<float> sediment_history;
           std::vector<float> material_stack_height;
           sd->getData(SimulationDataEnum::sedimentation_history,sediment_history);
           sd->getData(SimulationDataEnum::material_stack_height,material_stack_height);
           glm::vec3 initial_sedimentation_point;
           glm::vec3 pp;
           sd->getData(SimulationDataEnum::initial_sedimentation_point,initial_sedimentation_point);
           sd->getData(SimulationDataEnum::actual_point,pp);
           if(sediment_history.size()>0 && material_stack_height.size()>0){
               MyMesh::Point point = _mesh.point(*vertex_iterator);
               glm::vec3 actual_point (point[0],point[1],point[2]);
               std::vector<float> tmp_sediment_history;
               std::vector<float> tmp_material_stack_height;
               for(float entry : sediment_history){
                   if(entry == 0)
                       break;
                   else{
                       tmp_sediment_history.push_back(entry);
                        }
               }
               for(float entry : material_stack_height){

                       tmp_material_stack_height.push_back(entry);

               }
               sedimentationData sdtmp = sedimentationData(initial_sedimentation_point,tmp_sediment_history,tmp_material_stack_height);

            selected_vertices.insert(std::make_pair(vertex_iterator,sdtmp));
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
    std::vector<glm::vec3> intermediate_points;
    std::vector<float> intermediate_materials;
         AShader* sedimentation_shader;
    if(sedimenation_data.sediment_history.size()==1)   {

        if(actual_point[2]- sedimenation_data.initial_position[2] > 0.01  ){


            intermediate_points.push_back(actual_point);
            intermediate_materials.push_back(sedimenation_data.sediment_history[0]);
//            intermediate_materials.push_back(0.0f);

            intermediate_points.push_back(sedimenation_data.initial_position);
            intermediate_materials.push_back(sedimenation_data.sediment_history[0]);

//            intermediate_materials.push_back(0.0f);
            /*
            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, 0));
            list_of_sedimentation_points.push_back(actual_point);


            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, sedimenation_data.sediment_history[0]));
            list_of_sedimentation_points.push_back(sedimenation_data.initial_position);*/
        }

    }else   {
        glm::vec3 initial_point = sedimenation_data.initial_position;
        glm::vec3 distance_vector =   actual_point - initial_point;
        int num_divisions =    sedimenation_data.sediment_history.size() -1 ;//- 2 + 1;
        float mod = sqrtf(dot(distance_vector,distance_vector));
        float step_length = mod / (float) num_divisions;
        glm::vec3 normalized_distance_vector = distance_vector/mod;
        glm::vec3 new_sedimentation_point;

//        for(uint i = 1 ;i<sedimenation_data.sediment_history.size()-1;i++){
            for(uint i = 0 ;i<sedimenation_data.sediment_history.size();i++){
            //new_sedimentation_point=  initial_point + normalized_distance_vector * step_length * (float )i ;
            new_sedimentation_point=  initial_point;
            new_sedimentation_point[2] += sedimenation_data.material_stack_width[i] ;//* (float )i ;

            intermediate_points.push_back(new_sedimentation_point);
            if(actual_point[2]-initial_point[2]<0)
            {
                intermediate_materials.push_back(0.0f);
            }else
             intermediate_materials.push_back(sedimenation_data.sediment_history[i]);
        }

    }
    if(intermediate_materials.size()>0){
        sedimentation_shader = new SedimentationShader(_id,1.0f,intermediate_points,intermediate_materials);
    list_of_sedimentation_points.push_back(actual_point);
    list_of_shaders.push_back(sedimentation_shader);
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

void SedimentationClassifier::AverageOutputData(){
    CreatePointCloud();
     for(uint i = 0;i<list_of_sedimentation_points.size();i++){
         glm::vec3 const entry = list_of_sedimentation_points[i];
         pcl::PointXYZLNormal new_point;
         int K = 15;//or 27
         new_point.x = entry[0];
         new_point.y = entry[1];
         new_point.z = entry[2];
         std::vector<int> pointIdxNKNSearch(K);
       //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
         std::vector<float> dists_list(K);
         std::vector<float> ids_list(K);
         float total_dist=0;
         float total= 0;
        float avg = 0;
        std::map<int,float> list_of_used_materials;
         std::vector<float> pointNKNSquaredDistance(K);
         if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
             for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 SedimentationShader* sd =  (SedimentationShader*) list_of_shaders[pointIdxNKNSearch[j]];
                 float val = sd->GetMaterialId();
               //  avg += val;
                 if(list_of_used_materials.empty()) {

                     list_of_used_materials.insert(std::make_pair(val,0));

                 }else{
                     if(list_of_used_materials.count(val)>0)
                     {
                         list_of_used_materials[val]++;
                     }else{
                         list_of_used_materials.insert(std::make_pair(val,0));
                     }

                 }
             }
             float max = -INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                if(entry.second>max){
                    max = entry.second;
                    final_id = entry.first;
                }
             }

            // float mean = avg/(float)pointIdxNKNSearch.size();
           /*  float min_dist = INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                 float dist =std::abs(entry.second-mean);
                 if(dist<min_dist){
                     min_dist = dist;
                    final_id = entry.first;
                 }
             }*/
            list_of_shaders[pointIdxNKNSearch[0]] = new SedimentationShader(_id,1.0f,final_id);
        }
    }
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
 /* CreatePointCloud();

     int _subdiv_levels = 0;
     for(int i=0;i<_subdiv_levels;i++){
         auto set_of_faces = GetSetOfFaces(selected_vertices);
         SubdividerAndInterpolator<MyMesh> catmull(set_of_faces);
             catmull.attach(_mesh);
             catmull(1);
             catmull.detach();
             selected_vertices = SelectSedimentationPoints();

    }

    AssignSedimentationParameters(selected_vertices);*/
   // AverageOutputData();
   // AverageData();
  AssignSedimentationParameters2(selected_vertices);
    list_of_points = list_of_sedimentation_points;
    list_of_data = list_of_shaders;
    details = 0.5;
}


