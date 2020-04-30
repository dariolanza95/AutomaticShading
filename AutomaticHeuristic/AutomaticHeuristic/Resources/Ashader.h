#ifndef ASHADER_H
#define ASHADER_H

#include <stdlib.h>
#include <vector>
#include <string>

enum class  BlendMode{Add,Overlay};

class AShader
{

    const int  _id ;
    float _confidence;
    BlendMode _displ_blend_mode;
public:
    AShader();
    AShader(int id);
   // AShader(int _id,BlendMode displ_blend_mode = BlendMode::Overlay);
    AShader(int _id,float _confidence,BlendMode displ_blend_mode = BlendMode::Overlay );
    float GetConfidence();
    void SetConfidence(float new_confidence);
    BlendMode GetDisplBlendMode();
    int GetId();
    virtual std::string getShaderName() = 0;
    virtual void getCloudPathName(std::string& path) = 0;
    virtual void getSerializedData(std::vector<float> &data) = 0;
    virtual void allocateData(std::vector<float> &data) = 0;
    virtual void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)=0;

};

#endif // ASHADER_H
