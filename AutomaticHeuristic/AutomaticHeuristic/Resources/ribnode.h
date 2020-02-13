#ifndef RIBNODE_H
#define RIBNODE_H
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
class RIBNode
{
public:
    RIBNode(string name, string color_output_name) ;

    string _name;
    string _color_output_name;
    string _displace_output_name;
};

#endif // RIBNODE_H
