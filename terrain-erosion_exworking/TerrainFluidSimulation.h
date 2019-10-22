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
#include <iostream>
#include <fstream>
#include "Graphics/Shader.h"
#include "Graphics/VertexBuffer.h"
#include "Graphics/IndexBuffer.h"
#include "Graphics/Texture2D.h"

#include "Camera.h"

#include "SimulationState.h"

#if defined(__APPLE__) || defined(__MACH__)
#include "osx_bundle.h"
#endif

#include <memory>

class TerrainFluidSimulation
{
public:
    TerrainFluidSimulation(GLFWwindow* window,uint dim=200);

    void Run();

    void Stop();


    void ExportSimulationInObjFormat();
    void  SaveTerrain(std::fstream *objfile) ;
protected:

    /// Starts simulation main loop.
    void runMainloop();

    /// Checks and handles input events.
    void checkInput();

    /// Advances physics by timestep dt (in milliseconds).
    void updatePhysics(double dt);

    /// Renders the simulation
    void render();

    void cameraMovement(double dt);

    void init();

    bool IsInPause();
protected:
    bool _finished;

    bool _rain;
    bool _flood;
    bool _inPause;

    GLFWwindow *_window;

    glm::vec2 _rainPos;

    SimulationState _simulationState;
    Simulation::FluidSimulation _simulation;


    Graphics::ShaderManager             _shaderManager;
    Graphics::VertexBuffer<float>       _terrainHeightBuffer;
    Graphics::VertexBuffer<float>       _waterHeightBuffer;
    //Vector for remember just x,y coordinates of the vertices
    Graphics::VertexBuffer<glm::vec2>   _gridCoordBuffer;
    //Useful to remember in which order the vertices are supposed to be connected
    Graphics::IndexBuffer               _gridIndexBuffer;
    Graphics::VertexBuffer<float>       _sedimentBuffer;
    Graphics::VertexBuffer<glm::vec3>   _normalBuffer;


    Graphics::Texture2D<float,Graphics::TextureFormat::Float,32> _waterHeightTexture;

    Camera _cam;

    std::shared_ptr<Graphics::Shader> _testShader;

    std::vector<uint> gridIndices;

    int _width, _height;



};

#endif // TERRAINFLUIDSIMULATION_H
