#ifndef RIBADDNODE_H
#define RIBADDNODE_H

#include "ribnode.h"
#include "ribconstant.h"
#include <sstream>
#include "Ashader.h"

class RIBAddNode:public RIBNode
{
    static int _unique_id;
    int _id;
    RIBNode* _shader_1;
    RIBNode* _shader_2;
    RIBNode* _mix;
    BlendMode _blend_mode;

public:
    RIBAddNode(RIBNode* shader1,RIBNode* shader2,BlendMode blend_mode = BlendMode::Add);
    RIBAddNode(RIBNode *shader1, RIBNode *shader2,RIBNode *mix, BlendMode blend_mode);
    std::string GetName();
    std::string WriteNode();
};

#endif // RIBADDNODE_H
