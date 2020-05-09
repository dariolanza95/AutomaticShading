#ifndef DISPLNODE_H
#define DISPLNODE_H

#include "ribnode.h"
#include "ribconstant.h"
#include "bxdfnode.h"
class DisplNode : public RIBNode
{
    static int _shared_id;
    int _unique_id;
    BXDFNode* _bxdf_node;
    RIBNode* _reference_node;
    float _displ_bound;
public:
    DisplNode(BXDFNode* bxdf_node,RIBNode* reference_node,float displ_bound);
    float GetDisplBound();
    std::string WriteNode();
    std::string GetName();
};

#endif // DISPLNODE_H
