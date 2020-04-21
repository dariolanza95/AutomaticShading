#ifndef MATERIALSHADER_H
#define MATERIALSHADER_H
#include "Ashader.h"
#include "utils.h"
#include <stdlib.h>
#include <iostream>
#include <string>

class MaterialShader : public AShader
{
    float hardness;
public:
    MaterialShader();
    MaterialShader(int id);

    MaterialShader(int id,float confidence,float hardness);
    void allocateData(float** data);
    void getSerializedData(float **data);
    void getSerializedTypes(char ***types, char ***variables_names, int *num_variables) ;
};

#endif // MATERIALSHADER_H
