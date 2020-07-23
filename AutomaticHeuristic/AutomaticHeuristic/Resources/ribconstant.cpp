#include "ribconstant.h"

RIBConstant::RIBConstant(glm::vec3 vector):color(vector){}
RIBConstant::RIBConstant(float value):val(value){color = glm::vec3(NAN);}

std::string RIBConstant::WriteNode(){
    return "";
}


std::string RIBConstant::GetName(){
    std::stringstream res;
    if(glm::any(glm::isnan(color))){
        res << val;
    }
    else{
        for(int i = 0;i<3;i++){
            res<<color[i]<<" ";
        }
    }
    return res.str();
}
