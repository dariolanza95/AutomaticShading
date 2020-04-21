#ifndef ASHADER_H
#define ASHADER_H


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
    virtual void getSerializedData(float** data) = 0;
    virtual void getSerializedTypes(char*** types,char*** variables_names,int *num_variables) = 0;
    virtual void allocateData(float** data) = 0;

};

#endif // ASHADER_H
