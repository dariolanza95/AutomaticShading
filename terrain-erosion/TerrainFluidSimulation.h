/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef TERRAINFLUIDSIMULATION_H
#define TERRAINFLUIDSIMULATION_H



#include "Simulation/FluidSimulation.h"

#include "Graphics/Shader.h"
#include "Graphics/VertexBuffer.h"
#include "Graphics/IndexBuffer.h"
#include "Graphics/Texture2D.h"
# define GLM_ENABLE_EXPERIMENTAL


#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "Camera.h"


#include "SimulationState.h"

#if defined(__APPLE__) || defined(__MACH__)
#include "osx_bundle.h"
#endif

#include <memory>

class TerrainFluidSimulation
{
public:
    TerrainFluidSimulation(GLFWwindow *_window, uint dim=200);

    void Run();

    void Stop();

protected:

    /// Starts simulation main loop.
    void runMainloop();

    /// Checks and handles input events.
    void checkInput();

    /// Advances physics by timestep dt (in milliseconds).
    void updatePhysics(double dt, ulong time);

    /// Renders the simulation
    void render();

    void cameraMovement(double dt);

    void init();

    //Save the simulated terrain and the data collected during the simulation
    void ExportSimulation();

    // Export the simulated terrain to an obj file
    void SaveTerrain( std::fstream *objfile);

    //Save data collected during the simulation to an external file
    void SaveSimulationData(std::fstream *datafile);

    //computer IndexBuffer and VertexBuffer for the arrow used to debug
    void ArrowData( Grid2D<vec4>& arrowCoords, std::vector<uint>& arrowIndices,glm::vec3 startingPoint,glm::vec3 normal);

    //render the debug tool
    void RenderDebugTool();

protected:
    bool _finished;
    bool _inPause;
    bool _rain;
    bool _air;
    bool _flood;
    bool _debug_mode = false;
    bool _hardness_mode = false;
    glm::vec2 _rainPos;
   // std::vector<uint> gridIndices;
    SimulationState _simulationState;
    Simulation::FluidSimulation _simulation;


    Graphics::ShaderManager             _shaderManager;
    Graphics::VertexBuffer<float>       _terrainHeightBuffer;
    Graphics::VertexBuffer<float>       _waterHeightBuffer;
    Graphics::VertexBuffer<float>       _airHeightBuffer;
    Graphics::VertexBuffer<glm::vec2>   _gridCoordBuffer;
    Graphics::VertexBuffer<glm::vec4>   _arrowCoordBuffer;
    Graphics::IndexBuffer               _arrowIndexBuffer;
    Graphics::IndexBuffer               _gridIndexBuffer;
    Graphics::VertexBuffer<float>       _sedimentBuffer;
    Graphics::VertexBuffer<float>       _sedimentedTerrainBuffer;
    Graphics::VertexBuffer<float>       _simDataBuffer;
    Graphics::VertexBuffer<float>       _simDataBuffer_2;

    Graphics::VertexBuffer<glm::vec3>   _normalBuffer;


    Graphics::Texture2D<float,Graphics::TextureFormat::Float,32> _waterHeightTexture;

    Camera _cam;

    std::shared_ptr<Graphics::Shader> _testShader;

    std::shared_ptr<Graphics::Shader> _arrowShader;
    GLFWwindow *_window;
    int _width, _height;

};

#endif // TERRAINFLUIDSIMULATION_H
