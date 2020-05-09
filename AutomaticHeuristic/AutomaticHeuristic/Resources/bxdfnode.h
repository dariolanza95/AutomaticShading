#ifndef BXDFNODE_H
#define BXDFNODE_H


#include "ribnode.h"
#include "ribconstant.h"
#include "sstream"
class BXDFNode : public RIBNode
{
    static int _shared_id;
    int _unique_id;
    RIBNode* _node;
public:
    std::string GetMaterialId();
    BXDFNode(RIBNode *node);
    std::string GetName();
    std::string WriteNode();

};

#endif // BXDFNODE_H
