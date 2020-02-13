#ifndef RIBWRITER_H
#define RIBWRITER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include "ribnode.h"

using namespace std;

class RIBWriter
{
    ofstream rib_file ;
public:
    RIBWriter(string out_name_file);
    void MixNode(RIBNode node1,RIBNode node2,RIBNode node3);
    void MixNode();
    void Shader();
};

#endif // RIBWRITER_H
