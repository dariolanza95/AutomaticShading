#ifndef RIVERCLASSIFIERTESTER_H
#define RIVERCLASSIFIERTESTER_H
#include "riverclassifier.h"
#include <assert.h>
#include <math.h>
#include <glm/vec3.hpp>
using namespace std;

class RiverClassifierTester
{
public:
    RiverClassifierTester();
    void Test();
protected:

    RiverClassifier* rc;
    MyMesh mesh;
    void TestSelectBySlope();
    void AttachMockUpSimulationDataToAllVertices();
    void AttachMockUpSimulationToABox(int width,int height,float box_width,float box_height,float centerX,float centerY);
    void attachMockUpSimulationDataToCorners(int width,int height);
    void selectGridExternalCorners_testSelectFrontierFunction(int width,int height);
    void selectBox_testSelectFrontier(int width,int height,int box_width,int box_height,int centerX,int centerY);
    void frontierMadeFromGridExternalCorners_testBFSFunction( map<MyMesh::VertexHandle,float> frontier);
    void frontierMadeFromBox_testBFSFunction(int box_width,int box_height, map<MyMesh::VertexHandle,float> frontier);
    void IterateThroughMesh();
    MyMesh Cube();
    MyMesh Grid(int width,int height);
    MyMesh Grid(int width,int height,float angle);

    void TestSelectFrontier();
};

#endif // RIVERCLASSIFIERTESTER_H
