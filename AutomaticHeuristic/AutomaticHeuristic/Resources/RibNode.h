#ifndef RIBNODE_H
#define RIBNODE_H

#include <string>
#include <sstream>
class RIBNode
{
public:
    RIBNode();
    virtual std::string WriteNode() = 0;
    virtual std::string GetName() = 0;
};

#endif // RIBNODE_H
