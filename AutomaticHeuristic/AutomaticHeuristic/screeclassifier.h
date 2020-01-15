#ifndef SCREECLASSIFIER_H
#define SCREECLASSIFIER_H

#include "aclassifier.h"
#include <math.h>
#include <stdlib.h>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <iostream>
using namespace OpenMesh;
using namespace std;


class ScreeClassifier: public AClassifier
{
public:
    ScreeClassifier(MyMesh mesh);
    ScreeClassifier(MyMesh mesh,float repose_angle , float treshold);
    map<MyMesh::FaceHandle,float> ClassifyVertices();
protected:
    float _repose_angle;
    float _treshold ;
    MyMesh _mesh;
};

#endif // SCREECLASSIFIER_H
