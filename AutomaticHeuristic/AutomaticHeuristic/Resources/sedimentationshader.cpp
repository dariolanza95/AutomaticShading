#include "sedimentationshader.h"

SedimentationShader::SedimentationShader(int id, float confidence,float material_id) : AShader(id,confidence),material_id(material_id)
{}

SedimentationShader::SedimentationShader(int id, float confidence,float material_id,float second_id) : AShader(id,confidence),material_id(material_id),stack_id(second_id)
{

}
SedimentationShader::SedimentationShader(int id, float confidence, std::vector<glm::vec3> list_of_intermediate_sedimentation_points, std::vector<float> list_of_intermediate_sedimentation_materials, std::vector<int> list_of_intermediate_material_ids):
    AShader(id,confidence),material_id(material_id),
    list_of_intermediate_sedimentation_points(list_of_intermediate_sedimentation_points),
    list_of_intermediate_sedimentation_materials(list_of_intermediate_sedimentation_materials),
    list_of_intermediate_stack_ids(list_of_intermediate_material_ids)
{}


SedimentationShader::SedimentationShader(int id): AShader(id){}
//void SedimentationShader::allocateData(std::vector<float> &data){
//    data.resize(1);
//}

bool SedimentationShader::GetLineSedimentationStackId(int id,glm::vec3 actual_point,int &stack_id){
    float min_dist = INFINITY;
    bool found = false;
    glm::vec3 min_point;
    if(list_of_intermediate_stack_ids.size()!=list_of_intermediate_sedimentation_points.size()){
        std::cout<<"list sizes dont match"<<std::endl;
    }
    if(actual_point[0]==0 && actual_point[1] == 212)
        std::cout<<"poi"<<std::endl;

    for(int i=0;i<list_of_intermediate_stack_ids.size();i++){
          if(list_of_intermediate_sedimentation_materials[i]==id){
              float dist = dot(actual_point-list_of_intermediate_sedimentation_points[i],actual_point-list_of_intermediate_sedimentation_points[i]);
              if(dist<min_dist){
                  min_dist = dist;
                  found = true;
                  min_point = list_of_intermediate_sedimentation_points[i];
                  stack_id =list_of_intermediate_stack_ids[i];
              }
          }
      }
    return found;
}


bool SedimentationShader::GetLineId2(int material_id,glm::vec3 actual_point,int &stack_id,glm::vec3 &min_point,glm::vec3 &max_point){
    float min_dist = INFINITY;
    bool found = false;
    if(list_of_intermediate_stack_ids.size()!=list_of_intermediate_sedimentation_points.size()){
        std::cout<<"list sizes dont match"<<std::endl;
    }
  //  if(actual_point[0]==0 && actual_point[1] == 212)
//        std::cout<<"poi"<<std::endl;

    for(int i=0;i<list_of_intermediate_sedimentation_materials.size();i++){
          if(list_of_intermediate_sedimentation_materials[i]==material_id){
              float dist = dot(actual_point-list_of_intermediate_sedimentation_points[i],actual_point-list_of_intermediate_sedimentation_points[i]);
              if(dist<min_dist){
                  min_dist = dist;
                  found = true;
                  min_point = list_of_intermediate_sedimentation_points[i];
                  if(i<list_of_intermediate_sedimentation_materials.size()-1)
                    max_point = list_of_intermediate_sedimentation_points[i+1];
                  else{
                      max_point = min_point;
                      max_point[2]+=1;
                  }
//                  stack_id = -1;
//                  stack_id =list_of_intermediate_stack_ids[i];
                  stack_id = list_of_intermediate_sedimentation_materials[i];
              }
          }
      }
    if(max_point[2]<actual_point[2])
        found = false;
  //  max_point[2]+= 10;
  //  min_point[2]+= 10;
    return found;
}


bool SedimentationShader::GetLineId(int stack_id,glm::vec3 actual_point,glm::vec3 &min_point,glm::vec3 &max_point){
    float min_dist = INFINITY;
    bool found = false;
    if(list_of_intermediate_sedimentation_materials.size()!=list_of_intermediate_sedimentation_points.size()){
        std::cout<<"list sizes dont match"<<std::endl;
    }
   // if(actual_point[0]==0 && actual_point[1] == 212)
   //     std::cout<<"poi"<<std::endl;
    for(int i=0;i<list_of_intermediate_sedimentation_materials.size();i++){
          if(list_of_intermediate_stack_ids[i]==stack_id){
              float dist = dot(actual_point-list_of_intermediate_sedimentation_points[i],actual_point-list_of_intermediate_sedimentation_points[i]);
              if(dist<min_dist){
                  min_dist = dist;
                  found = true;
                  min_point = list_of_intermediate_sedimentation_points[i];
                  if(i<list_of_intermediate_stack_ids.size()-1)
                    max_point = list_of_intermediate_sedimentation_points[i+1];
                  else{
                      max_point = min_point;
                      max_point[2]+=1;
                  }
              }
          }
      }
    return found;
}

bool SedimentationShader::getClosestPointWithSameId(glm::vec3 actual_point,int id,glm::vec3& closest_point){

    float min_dist = INFINITY;
    uint index = -1;
    for(int i=0;i<list_of_intermediate_sedimentation_points.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        float temp_dist = dot(actual_point-point,actual_point-point);
        if(temp_dist<min_dist && id == list_of_intermediate_sedimentation_materials[i]){
            min_dist = temp_dist;
            index = i;
            if(i<list_of_intermediate_sedimentation_points.size()){
                glm::vec3 point = list_of_intermediate_sedimentation_points[i+1];
                float temp_dist = dot(actual_point-point,actual_point-point);
                if(temp_dist<min_dist){
                    min_dist = temp_dist;
                    index = i+1;
                }
            }
        }
    }
    if(index!=-1){
        closest_point = list_of_intermediate_sedimentation_points[index];
        return true;
    }else{
        return false;
    }
}

std::vector<glm::vec3> SedimentationShader::getListOfIntermediateSedimentationPoints() {return list_of_intermediate_sedimentation_points;}

SedimentationShader::~SedimentationShader(){}

uint SedimentationShader::findClosestPointInList(glm::vec3 actual_point,float& dist){

    float min_dist = INFINITY;
    uint index = -1;
    for(int i=0;i<list_of_intermediate_sedimentation_points.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        //float temp_dist = dot(actual_point-point,actual_point-point);
        float temp_dist = actual_point[2]-point[2];
//        if(temp_dist<min_dist)
        if(std::abs(temp_dist)<min_dist /*&& temp_dist<0*/)

        {
            min_dist = temp_dist;
            index = i;
        }
    }
   //float projection;
   // glm::vec3 projected_actual_point = ProjectAlongStackDirection(actual_point,projection);
   // if(projection<=0){
   //     //note that if the project is negative then the closest point has to be lowest in the stack
   //     return 0;
   // }

    //glm::vec3 initial_point = list_of_intermediate_sedimentation_points.front();
    //float distance_actual_point = sqrtf(dot(projected_actual_point-initial_point,projected_actual_point-initial_point));
    //float distance_closest_point = sqrtf(dot(list_of_intermediate_sedimentation_points[index]-initial_point,list_of_intermediate_sedimentation_points[index]-initial_point));
    /*if(distance_actual_point<distance_closest_point && index>0){
           index -= 1;
       }*/


//     if(distance_actual_point>distance_closest_point && index<list_of_intermediate_sedimentation_points.size()-1){
//        index += 1;
//    }
   glm::vec3 final_point= list_of_intermediate_sedimentation_points[index];

    dist = min_dist*dot(final_point-actual_point,final_point-actual_point);
    return index;
}

glm::vec3 SedimentationShader::ProjectAlongStackDirection(glm::vec3 actual_point,float& projection){
    glm::vec3 initial_point = list_of_intermediate_sedimentation_points.front();
    glm::vec3 last_point = list_of_intermediate_sedimentation_points.back();
    glm::vec3 direction = last_point-initial_point;
    direction = glm::normalize(direction);
    if(glm::any(glm::isnan(direction))){
        projection = 0;
        return actual_point;
    }
    glm::vec3 actual_point_under_new_system = actual_point-initial_point;
    projection = dot(actual_point_under_new_system,direction);
    return  projection*direction + initial_point;
}

float SedimentationShader::utilityFunct(glm::vec3 actual_point,float id,float& weight){
    if(actual_point[0] == 0 && actual_point[1] == 100)
        std::cout<<"PoI"<<std::endl;
    float projection;
    //float distance_from_the_initial_point = projection;// dot(projected_actual_point-initial_point,projected_actual_point-initial_point);
    glm::vec3 projected_actual_point = ProjectAlongStackDirection(actual_point,projection);
    if(projection<=0)
        return 0;
    glm::vec3 initial_point = list_of_intermediate_sedimentation_points.front();
    float min_dist = INFINITY;
    float max_dist = -INFINITY;
    int same_id_index = -1;
    int index_closest_point = 0;
    for(int i = 0;i<list_of_intermediate_sedimentation_points.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        float dist = dot(point-projected_actual_point,point-projected_actual_point);
        if(dist<min_dist){
            min_dist = dist;
            index_closest_point = i;
        }

            if(max_dist<dist){
                max_dist = dist;
            }

    }
float eps = 0.0001;
    glm::vec3 closest_point = list_of_intermediate_sedimentation_points[index_closest_point];
    float dist_from_local_origin =  sqrtf(dot(closest_point-initial_point,closest_point-initial_point));
    weight = dot(closest_point-actual_point,closest_point-actual_point);
//    weight = std::max(weight,eps);

    if(projection>dist_from_local_origin  && index_closest_point<list_of_intermediate_sedimentation_points.size()-1){
        //- eps &&  dist_from_local_origin<projection + eps
    //we are in the upper part of the stack
        index_closest_point+=1;
    }else{
        if(list_of_intermediate_sedimentation_materials[index_closest_point]==id){
            return 0;
        }/*else{
            if(dist_from_local_origin==projection && projection>0)
            index_closest_point +=1;
        }*/
    }
    if(list_of_intermediate_sedimentation_materials[index_closest_point]==id){
        //the closest point has also the same id
        return 0;
    }
    min_dist = sqrtf(min_dist);
    float minimal_distance_from_point_with_same_id = INFINITY;

    for(int i = 0;i<list_of_intermediate_sedimentation_materials.size();i++){

        if(list_of_intermediate_sedimentation_materials[i]==id){
            same_id_index = i;
            glm::vec3 base_of_sediment_layer = list_of_intermediate_sedimentation_points[i];
            float dist_from_base =sqrtf(dot(base_of_sediment_layer-projected_actual_point,base_of_sediment_layer-projected_actual_point));
            if(i!=0/*list_of_intermediate_sedimentation_materials.size()-1*/){
                glm::vec3 end_of_sediment_layer;
                end_of_sediment_layer = list_of_intermediate_sedimentation_points[i-1];
                float dist_from_end = sqrtf(dot(end_of_sediment_layer-projected_actual_point,end_of_sediment_layer-projected_actual_point));
               // dist_from_base = std::max(dist_from_base,eps);
               // dist_from_end =  std::max(dist_from_end,eps);
                if(dist_from_end<dist_from_base)
                    minimal_distance_from_point_with_same_id = dist_from_end;//sqrtf(dot()) dist_from_end;
                else{
                    minimal_distance_from_point_with_same_id = dist_from_base;
                }
            }else{
                minimal_distance_from_point_with_same_id = dist_from_base;
            }
        }

    }


    if(same_id_index!=-1){
       /* if(minimal_distance_from_point_with_same_id-min_dist == 0){
            return 0;
        }
        if(minimal_distance_from_point_with_same_id-min_dist<0)
            std::cout<<"we have a problem"<<std::endl;*/
        return minimal_distance_from_point_with_same_id;//-min_dist;//1/((minimal_distance_from_point_with_same_id-min_dist)*(minimal_distance_from_point_with_same_id-min_dist));
    }else{
        return  max_dist;//1+1/((minimal_distance_from_point_with_same_id-min_dist)*(minimal_distance_from_point_with_same_id-min_dist));

//        return max_dist;
    }

}

/*float SedimentationShader::utilityFunct(glm::vec3 actual_point,float id){
    float min_dist = INFINITY;
    float closest_dist = INFINITY;

    float max_dist = -INFINITY;

    int index = -1;
    int index_closest_point = -1;
    float dist ;

    for(int i=0;i<list_of_intermediate_sedimentation_materials.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        dist = dot(actual_point-point,actual_point-point);
        if(dist<closest_dist){
            closest_dist = dist;
            index_closest_point = i;
        }
        if(list_of_intermediate_sedimentation_materials[i] == id){
           if(dist<min_dist){
               min_dist = dist;
               index = i;
           }

        }
        if(dist>max_dist){
            max_dist = dist;
        }

    }
    if(min_dist!=INFINITY){

        return min_dist;//-closest_dist;
    }
    else
        return max_dist;
    /*for(int i=0;i<list_of_intermediate_sedimentation_points.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        dist = dot(actual_point-point,actual_point-point);
        //dist= abs(actual_point[2]-point[2]);
        if(dist<min_dist){
            min_dist = dist;
            index = i;
        }
    }
    if(id == list_of_intermediate_sedimentation_materials[index]){
        return dist ;
    }else{
        if(index > 0 )
            if(id == list_of_intermediate_sedimentation_materials[index-1]){
                dist = dot(actual_point-list_of_intermediate_sedimentation_points[index-1],actual_point-list_of_intermediate_sedimentation_points[index-1]);
                return dist;
            }
        else{
            if(index < list_of_intermediate_sedimentation_materials.size()-1)
                if( id == list_of_intermediate_sedimentation_materials[index+1]){
                    dist = dot(actual_point-list_of_intermediate_sedimentation_points[index+1],actual_point-list_of_intermediate_sedimentation_points[index+1]);
                    return dist;

                }
        }
    }
    return dist*5;*/
//}

std::vector<float> SedimentationShader::getListOfIntermediateSedimentationMaterials() {return list_of_intermediate_sedimentation_materials;}

float SedimentationShader::getClosestPointMatId(glm::vec3 actual_point,float& dist){
    int index = findClosestPointInList(actual_point,dist);
    //if(list_of_intermediate_sedimentation_points.back()[2]< actual_point[2]){
    //    dist = INFINITY;
    //    return list_of_intermediate_sedimentation_materials.back();
    //}
    return list_of_intermediate_sedimentation_materials[index];
}



int SedimentationShader::getClosestPointIndex(glm::vec3 actual_point){
    float min_dist = INFINITY;
    int index = -1;
    for(int i=0;i<list_of_intermediate_sedimentation_points.size();i++){
        glm::vec3 point = list_of_intermediate_sedimentation_points[i];
        glm::vec3 dist_vector = actual_point -  point;
        float dist = glm::dot(dist_vector,dist_vector);
       // float dist = abs(actual_point[2] - point[2]);
        if(dist<min_dist){
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

float SedimentationShader::GetMaterialId(){
    return material_id;
}



float SedimentationShader::GetStackId(){
    return stack_id;
}
float SedimentationShader::GetMaterialId(SedimentationShader sd1,glm::vec3 actual_point){
    float treshold = 0.1;
    float id;
    size_t index = -1;
     float min_dist = INFINITY;
    int diff_size_lists = sd1.list_of_intermediate_sedimentation_points.size() - list_of_intermediate_sedimentation_points.size();
    for(uint j = 0;j<sd1.list_of_intermediate_sedimentation_points.size()-diff_size_lists;j++){
       glm::vec3 intersection_point;
      // std::cout<<j<<std::endl;
       intersection_point = glm::closestPointOnLine(actual_point,sd1.list_of_intermediate_sedimentation_points[j],list_of_intermediate_sedimentation_points[j]);
       glm::vec3 dist_vector = intersection_point-actual_point;
       float dist = sqrtf(glm::dot(dist_vector,dist_vector));
       if(dist<min_dist &&  list_of_intermediate_sedimentation_materials[j] == sd1.list_of_intermediate_sedimentation_materials[j]){
            min_dist = dist;
            index = j;
       }
    }
  /*  if(min_dist>treshold){
        float dist;
        if(diff_size_lists>0){
            index = findClosestPointInList(actual_point,dist);
        }else{
            if(diff_size_lists<0){
                index = findClosestPointInList(actual_point,list_of_intermediate_sedimentation_points,dist);
            }else
                return 0.0f;
        }
    }else{
        return list_of_intermediate_sedimentation_materials[index];
    }*/
    return list_of_intermediate_sedimentation_materials[index];
}


void SedimentationShader::getSerializedData(std::vector<float>& data){
    data.resize(1);
    data[0] = material_id;
}
std::string SedimentationShader::getShaderName(){
    return "SedimentationShader";
}

void SedimentationShader::getCloudPathName(std::string& path){
    std::string temp = std::string("pointcloud_SedimentationShader");
    path = temp;
}
void SedimentationShader::getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)
{
    types.resize(1);
    var_names.resize(1);
    num_variables = 1;
    for(int i = 0;i<1;i++){
        types[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }

    std::stringstream strm;
    for(int i = 0;i<1;i++){
        strm<<"shader_parameter_"<<i;
        var_names[i] = AutomaticShaders::Utils::fromStringToChar(strm.str());
        strm.str("");
    }
}
