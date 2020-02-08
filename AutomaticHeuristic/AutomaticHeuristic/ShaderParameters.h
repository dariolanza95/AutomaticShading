#ifndef SHADERPARAMETERS_H
#define SHADERPARAMETERS_H

#include <vector>

class ShaderParameters
{
   const int  _id ;
    std::vector<float> _list;
public:

    ShaderParameters();
    ShaderParameters(int id,int size);
    int getId();
    int getParametersListSize();
    float getValue(int index);
    void setValue(int index,float value);

};



#endif // SHADERPARAMETERS_H
