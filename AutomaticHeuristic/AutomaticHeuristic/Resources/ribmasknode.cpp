#include "ribmasknode.h"

using namespace std;

int RIBMaskNode::_unique_id = 0;
RIBMaskNode::RIBMaskNode(std::shared_ptr<AShader> shader) : _shader(shader)
{
    _id = _unique_id++;
}
string RIBMaskNode::GetName(){
    stringstream res;
    res << "MaskReader"<<_id;
    return res.str() ;

}

string RIBMaskNode::WriteNode(){
    string cloud_file_name;
    _shader->getCloudPathName(cloud_file_name);
    cloud_file_name+"_mask";
    stringstream res;
    res<< "Pattern \"MaskReader\" \"MaskReader"<< _id <<
            "\" \"string filename\" [\""<<cloud_file_name<<"_mask\"]";
    return res.str();
}
