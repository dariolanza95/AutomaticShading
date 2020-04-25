#ifndef RIBSHADERNODE_H
#define RIBSHADERNODE_H
#include <string>
#include <sstream>
#include "Ashader.h"
#include "ribnode.h"
class RIBShaderNode : public RIBNode
{
    std::string _node_name;
    std::string _color_output_name;
    std::string _displace_output_name;
    std::string _float_output_name;
    AShader* _shader;
    int _id;
    static int _unique_id;
public:
    RIBShaderNode(AShader* shader);
    std::string GetName();
    std::string WriteNode();
    std::string GetDispl();
    std::string GetRGB()    ;
    std::string GetResultF();
};
#endif // RIBSHADERNODE_H
