#include "sedimentationshader.h"

SedimentationShader::SedimentationShader(int id, float confidence,float material_id) : AShader(id,confidence),material_id(material_id)
{

}

SedimentationShader::SedimentationShader(int id,float confidence,std::vector<glm::vec3> list_of_intermediate_sedimentation_points,std::vector<float> list_of_intermediate_sedimentation_materials):
    AShader(id,confidence),material_id(material_id),list_of_intermediate_sedimentation_points(list_of_intermediate_sedimentation_points),list_of_intermediate_sedimentation_materials(list_of_intermediate_sedimentation_materials)
{}


SedimentationShader::SedimentationShader(int id): AShader(id){}
void SedimentationShader::allocateData(std::vector<float> &data){
    data.resize(1);
}

std::vector<glm::vec3> SedimentationShader::getListOfIntermediateSedimentationPoints() {return list_of_intermediate_sedimentation_points;}


uint findClosestPointInList(glm::vec3 actual_point, std::vector<glm::vec3> list_of_points,float& dist){

    float min_dist = INFINITY;
    uint index = -1;
    for(int i=0;i<list_of_points.size();i++){
        glm::vec3 point = list_of_points[i];
       // float dist = abs(actual_point[2] -  point[2]);
        float dist = dot(actual_point-point,actual_point-point);
        if(dist<min_dist){
            min_dist = dist;
            index = i;
        }
    }
    dist = min_dist;
    return index;
}

float SedimentationShader::utilityFunct(glm::vec3 actual_point,float id){
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
}

std::vector<float> SedimentationShader::getListOfIntermediateSedimentationMaterials() {return list_of_intermediate_sedimentation_materials;}

float SedimentationShader::getClosestPointMatId(glm::vec3 actual_point,float& dist){
    int index = findClosestPointInList(actual_point,this->list_of_intermediate_sedimentation_points,dist);
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
    if(min_dist>treshold){
        float dist;
        if(diff_size_lists>0){
            index = findClosestPointInList(actual_point,sd1.list_of_intermediate_sedimentation_points,dist);
        }else{
            if(diff_size_lists<0){
                index = findClosestPointInList(actual_point,list_of_intermediate_sedimentation_points,dist);
            }else
                return 0.0f;
        }
    }else{
        return list_of_intermediate_sedimentation_materials[index];
    }
    return list_of_intermediate_sedimentation_materials[index];
}


void SedimentationShader::getSerializedData(std::vector<float>& data){
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
    for(int i = 0;i<4;i++){
        strm<<"shader_parameter_"<<i;
        var_names[i] = AutomaticShaders::Utils::fromStringToChar(strm.str());
        strm.str("");
    }
}
