#include "ribmixnode.h"


using namespace std;

int RIBMixNode::_unique_id = 0;
RIBMixNode::RIBMixNode(RIBNode* mix,RIBNode* shader_1,RIBNode* shader_2,bool mix_floats) :
    _mix(mix),_shader_1(shader_1),_shader_2(shader_2),_mix_floats(mix_floats)
{_id = _unique_id++;}

string RIBMixNode::GetName(){
stringstream res;
res << "PxrMix"<<_id;
return res.str() ;

}
string RIBMixNode::WriteMixFloats(){
    stringstream string_to_write;
    string_to_write<< "Pattern\"PxrMix\"";

    stringstream name;
    name<< "PxrMix"<<_id;
    string_to_write<< " \""<< name.str() << "\" ";
    std::cout<<"mask: "<<_mix->GetName()<<std::endl;
    if(dynamic_cast<RIBConstant*>(_mix))
    {
        string_to_write<< "\"float mix \""<< "["<<_mix->GetName()<<"]";
    }
    else
    {
        string_to_write<< " \"reference  float mix\" [\""<< _mix->GetName()<< ":resultF\"] ";
    }
    if(dynamic_cast<RIBConstant*>(_shader_1))
    {
        string_to_write << " \"float resultR\" [ "<< _shader_1->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference float resultR\" [ \""<<_shader_1->GetName()<<":resultDispl\" ]";
    }

    if(dynamic_cast<RIBConstant*>(_shader_2))
    {
        string_to_write << " \"float resultR\" [ "<< _shader_2->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference float resultR\" [ \""<<_shader_2->GetName()<<":resultDispl\" ]";
    }
    string_to_write << " \"int clampMix\" [0]  "<<endl;
    return string_to_write.str();

}

string RIBMixNode::WriteMixColor(){
    stringstream string_to_write;
    string_to_write<< "Pattern\"PxrMix\"";

    stringstream name;
    name<< "PxrMix"<<_id;
    string_to_write<< " \""<< name.str() << "\" ";
    std::cout<<"mask: "<<_mix->GetName()<<std::endl;
    if(dynamic_cast<RIBConstant*>(_mix))
    {
        string_to_write<< "\"float mix \""<< "["<<_mix->GetName()<<"]";
    }
    else
    {
        string_to_write<< " \"reference  float mix\" [\""<< _mix->GetName()<< ":resultF\"] ";
    }
    if(dynamic_cast<RIBConstant*>(_shader_1))
    {
        string_to_write << " \"color color1\" [ "<< _shader_1->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference color color1\" [ \""<<_shader_1->GetName()<<":resultRGB\" ]";
    }

    if(dynamic_cast<RIBConstant*>(_shader_2))
    {
        string_to_write << " \"color color2\" [ "<< _shader_2->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference color color2\" [ \""<<_shader_2->GetName()<<":resultRGB\" ]";
    }
    string_to_write << " \"int clampMix\" [0]  "<<endl;
    return string_to_write.str();

}

string RIBMixNode::WriteNode(){
    if(_mix_floats)
        return WriteMixFloats();
    else
        return WriteMixColor();
}
