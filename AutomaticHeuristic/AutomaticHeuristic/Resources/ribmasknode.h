#ifndef RIBMASKNODE_H
#define RIBMASKNODE_H

#include "Ashader.h"
#include "RibNode.h"
#include "sstream"
class RIBMaskNode : public RIBNode
{
    AShader* _shader;
    static int _unique_id;
    int _id;
public:
    RIBMaskNode(AShader* shader);
    std::string WriteNode();
    std::string GetName();
};

#endif // RIBMASKNODE_H
