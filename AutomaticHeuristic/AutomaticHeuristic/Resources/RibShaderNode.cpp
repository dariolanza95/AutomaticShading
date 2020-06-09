#include "ribshadernode.h"

using namespace  std;
int RIBShaderNode::_unique_id = 0;
/*RIBShaderNode::RIBShaderNode(string name, string color_output_name):_node_name(name),_color_output_name(color_output_name)
{_id = _unique_id++;}
*/
RIBShaderNode::RIBShaderNode(std::shared_ptr<AShader> shader):_shader(shader){
    _id = _unique_id++;
    _color_output_name = "";
    _displace_output_name = "";
    _float_output_name = "";

}


std::string RIBShaderNode::GetName(){
    stringstream res;
    res << _shader->getShaderName()<<_id;
    return res.str() ;
}

std::string RIBShaderNode::WriteNode(){
    stringstream res;
    string name = _shader->getShaderName();
    res<< "Pattern \""<< name << "\" ";
    res<< " \""<<name<< _id<<"\"";
    return res.str();
}

string RIBShaderNode::GetDispl(){return _displace_output_name;}
string RIBShaderNode::GetRGB(){return _color_output_name;}
string RIBShaderNode::GetResultF(){return _float_output_name;}
