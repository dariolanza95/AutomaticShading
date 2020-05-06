#ifndef OPENGLVISUALIZER_H
#define OPENGLVISUALIZER_H


#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <inttypes.h>
#include <cassert>
#include "Resources/ShaderWrapper.h"
#include "Graphics/Shader.h"
#include "Graphics/VertexBuffer.h"
#include "Graphics/IndexBuffer.h"
#include "Camera.h"
#include <memory>
#include "Resources/simulationdata.h"
#include "platform_includes.h"
//#include <glm/glm.hpp> // brew install glm
//#include <GL/glew.h> // brew install glew
//
//#include <GL/glx.h>
//#include "Graphics/Grid2D.h"
//#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL;
#include <glm/gtx/quaternion.hpp>

#include <glm/gtc/type_ptr.hpp>
#include "Graphics/PerlinNoise.h"
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>

#include <glm/gtx/string_cast.hpp>

#ifdef Success
  #undef Success
#endif
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace glm;
using namespace Graphics;
#include "./Resources/mydefwrapper.h"
//typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr MyCloudPtr;
class OpenGlVisualizer
{
private :
    Graphics::ShaderManager             _shaderManager;
    Graphics::VertexBuffer<float>       _terrainHeightBuffer;
    Graphics::VertexBuffer<float>       _waterHeightBuffer;
    Graphics::VertexBuffer<glm::vec3>   _gridCoordBuffer;
    Graphics::IndexBuffer               _gridIndexBuffer;
    Graphics::VertexBuffer<float>       _sedimentBuffer;
    Graphics::VertexBuffer<float>       _simDataBuffer;
    Graphics::VertexBuffer<glm::vec3>   _normalBuffer;
    Camera _cam;
    std::shared_ptr<Graphics::Shader> _testShader;
    MyMesh _mesh;
    int _width;
    int _height;
    int _grid_width;
    int _grid_height;
     GLFWwindow *_window;
    void InitializeOpengGL();
    void InitializeBuffers();
    void CameraMovement(float dt);
    int ClampX(int x);
    int ClampY(int y);
    void Render();
    void ShowSimulationDataInput();
    void ShowSelectedFaces();
    void ParseInputFile(std::vector<glm::vec3>& gridCoords,std::vector<uint>& gridIndices);
    Grid2D<float> _terrain;
    Grid2D<float> _water;
    Grid2D<float> _suspendedSediment;
    Grid2D<float> _simData;
    Grid2D<vec3>  _surfaceNormals;
    std::string _obj_file;

public:
    OpenGlVisualizer(GLFWwindow* window,int width,int height,MyMesh mesh,std::string obj_file);
    void Initialize();
    void Visualize();

    Camera GetCamera();
};

#endif // OPENGLVISUALIZER_H
