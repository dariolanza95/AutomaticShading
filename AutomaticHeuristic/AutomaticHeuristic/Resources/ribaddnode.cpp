#include "ribaddnode.h"
using namespace std;
int RIBAddNode::_unique_id = 0;
RIBAddNode::RIBAddNode(RIBNode *shader1, RIBNode *shader2, BlendMode blend_mode): _shader_1(shader1),
    _shader_2(shader2),_blend_mode(blend_mode)
{
    _id = _unique_id++;
}

RIBAddNode::RIBAddNode(RIBNode *shader1, RIBNode *shader2,RIBNode *mix, BlendMode blend_mode): _shader_1(shader1),
    _shader_2(shader2),_mix(mix), _blend_mode(blend_mode)
{
    _id = _unique_id++;
}

std::string RIBAddNode::GetName(){
    stringstream res;
    res << "PxrAdd"<<_id;
    return res.str() ;
}

std::string RIBAddNode::WriteNode(){
    stringstream string_to_write;
    stringstream name;
    string_to_write<< "Pattern \"PxrAdd\"";

    name<< "PxrAdd"<<_id;
    string_to_write<< " \""<< name.str() << "\" ";
    if(dynamic_cast<RIBConstant*>(_shader_1))
    {
        string_to_write << " \"float float1\" [ "<< _shader_1->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference float float1\" [ \""<<_shader_1->GetName()<<":resultF\" ]";
    }

    if(dynamic_cast<RIBConstant*>(_shader_2))
    {
        string_to_write << " \"float float2\" [ "<< _shader_2->GetName() << " ]";
    }
    else
    {
        string_to_write<<" \"reference float float2\" [ \""<<_shader_2->GetName()<<":resultF\" ]";
    }
    if(_blend_mode == BlendMode::Add){
        string_to_write<<" \"int addMode\" [ "<< 1 << " ]";
    }else{
        string_to_write<<" \"int addMode\" [ "<< 0 << " ]";
        string_to_write<<" \"reference float mix\" [ \""<< _mix->GetName() <<":resultF\" ]";
    }

    string_to_write << " \"int clampMix\" [0]  "<<endl;
    return string_to_write.str();


}
