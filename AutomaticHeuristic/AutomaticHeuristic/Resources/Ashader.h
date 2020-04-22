#ifndef ASHADER_H
#define ASHADER_H

#include <stdlib.h>
#include <vector>
#include <string>
class AShader
{

    const int  id ;
    float confidence;
public:
    AShader();
    AShader(int id);
    AShader(int id,float confidence);
    float GetConfidence();
    void SetConfidence(float new_confidence);
    int GetId();

    virtual void getOutputCloudPath(std::string& path) = 0;
    virtual void getSerializedData(std::vector<float> &data) = 0;
    virtual void allocateData(std::vector<float> &data) = 0;
    virtual void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables)=0;

};

#endif // ASHADER_H
