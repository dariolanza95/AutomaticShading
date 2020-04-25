#include "displnode.h"

using namespace std;

int DisplNode::_shared_id = 0;

DisplNode::DisplNode(BXDFNode *bxdf_node, RIBNode *reference_node, float displ_bound):_bxdf_node(bxdf_node),_displ_bound(displ_bound),
  _reference_node(reference_node)
{
    _unique_id = _shared_id++;
}
string DisplNode::GetName(){
    stringstream name;
    name << "PxrDisplace"<<_unique_id;
    return name.str();
}


string DisplNode::WriteNode(){
    stringstream string_to_write;
string_to_write<<"Displace \"PxrDisplace\" \""<<GetName()<<"\" \"reference float dispScalar\" [\""<<_reference_node->GetName()<<":resultF\"]"<<std::endl;
string_to_write<<"\"reference float dispAmount\" [\""<<_reference_node->GetName()<<":resultF\"] \"int enabled\" [1]"<<std::endl;
string_to_write<<"\"vector dispVector\" [0 0 0] \"vector modelDispVector\" [0 0 0] \"string __materialid\" [\""<<_bxdf_node->GetMaterialId() <<"\"]"<<std::endl;
return string_to_write.str();
}

float DisplNode::GetDisplBound(){return _displ_bound;}
