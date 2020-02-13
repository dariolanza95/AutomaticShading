#ifndef OPENGLVISUALIZER_H
#define OPENGLVISUALIZER_H


#define GLFW_INCLUDE_GL3
#define GLFW_NO_GLU
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <inttypes.h>
#include <cassert>
#include "Resources/ShaderParameters.h"
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
#include "Graphics/PerlinNoise.h"
#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace glm;
using namespace Graphics;

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

class OpenGlVisualizer
{
private :
    Graphics::ShaderManager             _shaderManager;
    Graphics::VertexBuffer<float>       _terrainHeightBuffer;
    Graphics::VertexBuffer<float>       _waterHeightBuffer;
    Graphics::VertexBuffer<glm::vec2>   _gridCoordBuffer;
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
    void ParseInputFile(Grid2D<vec2>& gridCoords,std::vector<uint>& gridIndices);
    Grid2D<float> _terrain;
    Grid2D<float> _water;
    Grid2D<float> _suspendedSediment;
    Grid2D<float> _simData;
    Grid2D<vec3>  _surfaceNormals;
    string _obj_file;

public:
    OpenGlVisualizer(GLFWwindow* window,int width,int height,MyMesh mesh,string obj_file);
    void Initialize();
    void Visualize();
};

#endif // OPENGLVISUALIZER_H
