#include "bxdfnode.h"
using  namespace std;

int BXDFNode::_shared_id = 0;

BXDFNode::BXDFNode(RIBNode* node)
{
    if(node==nullptr)
        _node = new RIBConstant(0.0f);
    else
        _node = node;
    _unique_id = _shared_id++;
}

std::string BXDFNode::GetMaterialId(){
    stringstream materialID;
    materialID<<"PxrSurface"<<_unique_id<<"SG";
    return materialID.str();

}

std::string BXDFNode::GetName(){

    stringstream name;
    name<<"PxrSurface"<<_unique_id;
    return name.str();
}

std::string BXDFNode::WriteNode(){
    stringstream res;
    stringstream name;
    if(_node == nullptr)
        return "";
    name<<"PxrSurface"<<_unique_id;
    if(dynamic_cast<RIBConstant*>(_node)!=0){
        res<<"Bxdf \"PxrSurface\" \""<<GetName()<<"\" \"color diffuseColor\" ["<<_node->GetName()<<"] ";
    }else{
        res<<"Bxdf \"PxrSurface\" \""<<GetName()<<"\" \"reference color diffuseColor\" [\""<<_node->GetName()<<":resultRGB\"] ";
    }
    res<<"\"string __materialid\" [\""<<GetMaterialId()<<"\"]"<<std::endl;
    return res.str();

}

