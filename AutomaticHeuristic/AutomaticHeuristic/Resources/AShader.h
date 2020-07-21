#ifndef ASHADER_H
#define ASHADER_H

#include <stdlib.h>
#include <vector>
#include <string>
#include <memory>

#include <string.h>
enum class  BlendMode{Add,Overlay};

class AShader
{
    bool mask_pointcloud=false;
    const int  _id ;
    float _confidence;
    BlendMode _displ_blend_mode;

public:
    virtual void getSerializedData(std::vector<float> &data) = 0;
    virtual void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)=0;
    virtual void getCloudPathName(std::string& path) = 0;
    AShader();
    AShader(int id);
    virtual ~AShader();



   // AShader(int _id,BlendMode displ_blend_mode = BlendMode::Overlay);
    AShader(int _id,float _confidence,BlendMode displ_blend_mode = BlendMode::Overlay );
    float GetConfidence();
    void SetConfidence(float new_confidence);
    std::shared_ptr<AShader> GenerateMaskShader();
    BlendMode GetDisplBlendMode();
    int GetId();
    virtual std::string getShaderName() = 0;
     void getCloudPathNameInterface(std::string& path);
     void getSerializedDataInterface(std::vector<float> &data) ;
     void getSerializedTypesInterface(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);

};

#endif // ASHADER_H
