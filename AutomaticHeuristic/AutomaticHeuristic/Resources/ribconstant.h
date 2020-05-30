#ifndef RIBCONSTANT_H
#define RIBCONSTANT_H
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include "RibNode.h"
class RIBConstant : public RIBNode
{
    glm::vec3 color;
    float val;
public:
    RIBConstant(float val);
    RIBConstant(glm::vec3 vector);
    std::string WriteNode();
    std::string GetName();
};

#endif // RIBCONSTANT_H
