#ifndef RIBMIXNODE_H
#define RIBMIXNODE_H

#include "RibNode.h"
#include "ribshadernode.h"
#include "ribconstant.h"
#include <sstream>
class RIBMixNode : public RIBNode
{
    RIBNode* _mix;
    RIBNode* _shader_1;
    RIBNode* _shader_2;
    static int _unique_id;
    int _id;
    bool _mix_floats;
    std::string WriteMixColor();
    std::string WriteMixFloats();
public:
    RIBMixNode(RIBNode* mix, RIBNode* shader_1, RIBNode* shader_2, bool mix_floats= false);
    std::string WriteNode();
    std::string GetName();
};

#endif // RIBMIXNODE_H
