#include "apointcloudwriter.h"

APointCloudWriter::APointCloudWriter(std::vector<std::shared_ptr<AShader>> list_of_used_shaders,
                                     std::vector<std::shared_ptr<ShadersWrapper>> list_of_shaders_wrapper,
                                     std::vector<pcl::PointXYZL,Eigen::aligned_allocator<pcl::PointXYZL>>  list_of_used_points):
    list_of_used_shaders(list_of_used_shaders),
    list_of_shaders_wrapper(list_of_shaders_wrapper),
    list_of_used_points(list_of_used_points)
{

}

void APointCloudWriter::getSerializedDataInterface(std::vector<float> &data,std::shared_ptr<AShader> shader){
    if(write_pointcloud_mask){
        data.resize(1);
        data[0] = shader->GetConfidence();// _confidence;
    }else{
        shader->getSerializedData(data);
    }
}

void APointCloudWriter::getSerializedTypesInterface(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables,std::shared_ptr<AShader> shader)
{
    if(write_pointcloud_mask){
        var_names.resize(1);
        types.resize(1);
        std::string string_1("float");
        std::string string_2("mask");
        types[0] = strdup(string_1.c_str());
        var_names[0] = strdup(string_2.c_str());
        //char * my_other_str = strdup(some_const_str);
        //types[0] =(char*) string_1.c_str();//AutomaticShaders::Utils::fromStringToChar("float");
        //var_names[0] =(char*) string_2.c_str();//AutomaticShaders::Utils::fromStringToChar("mask");
        num_variables = 1;
    }else{
        shader->getSerializedTypes(types,var_names,num_variables);
    }
}


void APointCloudWriter::getCloudPathNameInterface(std::string& path, std::shared_ptr<AShader> shader){
shader->getCloudPathName(path);
if(write_pointcloud_mask){
    path = path+"_mask";
    }
}

void APointCloudWriter::WritePointClouds(){
    for(std::shared_ptr<AShader> shader:list_of_used_shaders){
        write_pointcloud_mask = true;
        WritePointCloud(shader);
        write_pointcloud_mask = false;
        WritePointCloud(shader);
    }
}
