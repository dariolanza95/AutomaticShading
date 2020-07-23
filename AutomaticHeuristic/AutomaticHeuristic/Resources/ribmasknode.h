#ifndef RIBMASKNODE_H
#define RIBMASKNODE_H

#include "AShader.h"
#include "RibNode.h"
#include "sstream"
#include <memory>
class RIBMaskNode : public RIBNode
{
    std::shared_ptr<AShader> _shader;
    static int _unique_id;
    int _id;
public:
    RIBMaskNode(std::shared_ptr<AShader> shader);
    std::string WriteNode();
    std::string GetName();
};

#endif // RIBMASKNODE_H
