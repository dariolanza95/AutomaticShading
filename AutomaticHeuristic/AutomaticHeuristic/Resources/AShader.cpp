#include "AShader.h"

//AShader::AShader(int id, float confidence = 0.0f, BlendMode displ_blend_mode): _id(id),_confidence(confidence) {}


AShader::AShader(int id): _id(id), _confidence(.0f),_displ_blend_mode(BlendMode::Overlay){}

AShader::AShader(int id,float confidence = 0.0f, BlendMode displ_blend_mode):_id(id), _confidence(confidence),_displ_blend_mode(displ_blend_mode){}

BlendMode AShader::GetDisplBlendMode(){return _displ_blend_mode;}

AShader::~AShader(){}


float AShader::GetConfidence(){return _confidence;}
void AShader::SetConfidence(float new_confidence){_confidence= new_confidence;}
int AShader::GetId(){return _id;}

//void AShader::SetMaskShader(){mask_pointcloud = true;}
//void AShader::UnSetMaskShader(){mask_pointcloud = false;}


void AShader::getCloudPathNameInterface(std::string& path){
    getCloudPathName(path);
    if(mask_pointcloud){
        path = path+"_mask";
    }
}


void AShader::getSerializedDataInterface(std::vector<float> &data){
    if(mask_pointcloud){
        data.resize(1);
        data[0] = _confidence;
    }else{
        getSerializedData(data);
    }
}
void AShader::getSerializedTypesInterface(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)
{
    if(mask_pointcloud){
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
        getSerializedTypes(types,var_names,num_variables);
    }
}

AShader* GenerateMaskShader(){

}

