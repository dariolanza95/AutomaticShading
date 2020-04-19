/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#include "TerrainFluidSimulation.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include "platform_includes.h"
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>

using namespace glm;
using namespace Graphics;


TerrainFluidSimulation::TerrainFluidSimulation(GLFWwindow* window, uint dim)
    : _simulationState(dim,dim),
      _simulation(_simulationState),
      _rain(false),
      _rainPos(dim/2,dim/2),
      _flood(false),
      _window(window),
      _cam(glm::vec3(0,0,0))
{}

void TerrainFluidSimulation::Run()
{
    init();
    runMainloop();
}

void TerrainFluidSimulation::Stop()
{
    _finished = true;
}

void TerrainFluidSimulation::runMainloop()
{
    using namespace std::chrono;

    // Settings
    double dt = 1000.0/(60); // 60 fps physics simulation

    // Setup
    high_resolution_clock clock;
    high_resolution_clock::time_point currentTime, newTime;
    high_resolution_clock::duration frameTime;
    high_resolution_clock::duration accumulator(0);

    // count number of simulation steps
    ulong counterSim = 0;

    _finished = false;
    currentTime = clock.now();
    while(!_finished)
    {
        newTime = clock.now();
        frameTime = newTime - currentTime;
        currentTime = newTime;
        accumulator += frameTime;

        if(! _inPause)
        {
            // physics simulation
            updatePhysics(dt);
        }

        // check for input events
        glfwPollEvents();
        cameraMovement(dt);

        // timing info
        counterSim++;
        if (counterSim%100 == 0)
        {
            double ms = std::chrono::duration_cast<milliseconds>(accumulator).count()/100.0;
            std::cout << 1000.0 /ms << " FPS\n";
            accumulator = high_resolution_clock::duration(0);
            counterSim = 0;
        }


        // input handling
        checkInput();

        // check for shader change
#if defined(__APPLE__) || defined(__MACH__)

#else
        _shaderManager.Update();
#endif
        // rendering
        render();
    }

}

void TerrainFluidSimulation::checkInput()
{
    // exit
    if (glfwGetKey(_window,GLFW_KEY_END))
    {
        _finished = true;
    }

    if (glfwGetKey(_window,'O')) _rain = true;
    if (glfwGetKey(_window,'P')) _rain = false;

    if (glfwGetKey(_window,'K')) _flood = true;
    if (glfwGetKey(_window,'L')) _flood = false;

    if (glfwGetKey(_window,'U')) _inPause = true;
    if (glfwGetKey(_window,'J')) _inPause = false;

    if (glfwGetKey(_window,'V'))
    {
     if(_inPause)
     {
        std::cout<<"Exporting"<<std::endl;
        TerrainFluidSimulation::ExportSimulation();
     }
     else
         std::cout<<"Can't save the simulation while it's running "<<std::endl;
    }

    // move rain position
    float d = 1.0f;
    if (glfwGetKey(_window,GLFW_KEY_UP)) _rainPos.y += d;
    if (glfwGetKey(_window,GLFW_KEY_DOWN)) _rainPos.y -= d;
    if (glfwGetKey(_window,GLFW_KEY_RIGHT)) _rainPos.x += d;
    if (glfwGetKey(_window,GLFW_KEY_LEFT)) _rainPos.x -= d;


    _simulation.rainPos = _rainPos;
}


void TerrainFluidSimulation::cameraMovement(double dt)
{
    float dtSeconds = dt/1000.0f;

    // camera control
    float camSpeed = 1.0f*dtSeconds;
    float rotSpeed = 60.0f*dtSeconds;

    vec3 yAxis(0,1,0);
    vec3 xAxis(1,0,0);
    vec3 zAxis(0,0,1);

    if (glfwGetKey(_window,'D')) _cam.TranslateLocal(vec3(camSpeed,0,0));
    if (glfwGetKey(_window,'A')) _cam.TranslateLocal(vec3(-camSpeed,0,0));
    if (glfwGetKey(_window,'R')) _cam.TranslateLocal(vec3(0,camSpeed,0));
    if (glfwGetKey(_window,'F')) _cam.TranslateLocal(vec3(0,-camSpeed,0));
    if (glfwGetKey(_window,'W')) _cam.TranslateLocal(vec3(0,0,-camSpeed));
    if (glfwGetKey(_window,'S')) _cam.TranslateLocal(vec3(0,0,camSpeed));
    if (glfwGetKey(_window,'Q')) _cam.GlobalRotate(yAxis,-rotSpeed);
    if (glfwGetKey(_window,'E')) _cam.GlobalRotate(yAxis,rotSpeed);
    if (glfwGetKey(_window,'T')) _cam.LocalRotate(xAxis,-rotSpeed);
    if (glfwGetKey(_window,'G')) _cam.LocalRotate(xAxis,rotSpeed);
    if (glfwGetKey(_window,'Z')) _cam.LocalRotate(zAxis,-rotSpeed);
    if (glfwGetKey(_window,'X')) _cam.LocalRotate(zAxis,rotSpeed);
    if (glfwGetKey(_window,'1')) _debug_mode = true;
    if (glfwGetKey(_window,'2')) _debug_mode = false;
    if (glfwGetKey(_window,'3')) _hardness_mode = true;
    if (glfwGetKey(_window,'4')) _hardness_mode = false;
}

void TerrainFluidSimulation::updatePhysics(double dt)
{
    // Run simulation
    _simulation.update(dt,_rain,_flood);

    // Copy data to GPU
    _terrainHeightBuffer.SetData(_simulationState.terrain);
    _waterHeightBuffer.SetData(_simulationState.water);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _simDataBuffer.SetData(_simulationState.simData);
    _normalBuffer.SetData(_simulationState.surfaceNormals);

}


void TerrainFluidSimulation::ExportSimulation()
{

           const char path_to_obj_file[ ] = "/home/pandora/thesis/AutomaticShading/Data/input.obj";
           const char path_to_simulation_data_file[ ] = "/home/pandora/thesis/AutomaticShading/Data/simulationData.txt";
           std::fstream objfile;
           std::fstream datafile;

           objfile.open(path_to_obj_file, std::fstream::out );
           datafile.open(path_to_simulation_data_file, std::fstream::out );

           SaveTerrain( &objfile);
           SaveSimulationData(&datafile);

           objfile.close();
           datafile.close();

}

float GetTerrainCapacity(float x,float y,float z ,float frequency,float _stratified_layer_width)
{

    PerlinNoise perlin;
    float kc = 0;
    float n = (perlin.Sample(frequency*x,frequency*y,frequency*z));

    //int res = _stratified_layer_width*(roundf(z/_stratified_layer_width));
    float level = sin((z+z/30*n)*_stratified_layer_width);
     if( level> 0)
     {
         kc = 35;
     }
     else
     {
         //if(level > 0)
         //{
         //   kc =28;
         //}
         //else
         {
             kc = 23;
         }

     }
     kc =kc + n*(kc/10);

return kc;
}


void TerrainFluidSimulation:: SaveSimulationData(std::fstream *datafile)
{
    int counter = 0;
    int temp = 0;
    std::string data_string;
    std::cout<<"Writing to some file"<<std::endl;

    for (uint y=0; y < _simulationState.vegetation.height(); y++)
    {
        for (uint x=0; x < _simulationState.vegetation.width(); x++)
        {
            temp++;
            //(*datafile )<< _simulationState.simData(x,y);
            //(*datafile )<<" "<< _simulationState.vegetation(x,y);
            float z = _simulationState.terrain(x,y);
            (*datafile)<< "hardness "<< GetTerrainCapacity(x,y,z,_simulation.noise_sediment_frequency,_simulation._stratified_layer_width);

            //vec3 flowNormal (_simulation.uVel(x,y),_simulation.vVel(x,y),_simulation.zVel(x,y));
            vec3 flowNormal(0,0,0);
            if(_simulation.count(x,y)>0)
            {
                flowNormal = vec3(_simulation.flowNormal(x,y)/_simulation.count(x,y));
                flowNormal = normalize(flowNormal);
            }

            (*datafile )<<" flow_normal v "<< flowNormal[0]  << " "<< flowNormal[1]<< " "<< flowNormal[2];
            (*datafile)<< std::endl;
        }
    }
}

void TerrainFluidSimulation:: SaveTerrain (std::fstream *objfile)
{
           float z = 0;
           for (uint y=0; y<_simulationState.terrain.height(); y++)
           {
               for (uint x=0; x<_simulationState.terrain.width(); x++)
               {
                   z = _simulationState.terrain(x,y);
                   (*objfile )<<"v " << x << " " << y << " "<< z << std::endl;
               }
           }
           std::vector<uint> gridIndices;
           Grid2DHelper::MakeGridIndicesClockWiseOrder(gridIndices,_simulationState.terrain.width(),_simulationState.terrain.height());
           std::cout<<"gridIndices.size() "<< gridIndices.size()<<std::endl;
           uint i = 0;
           for (i = 0;i<gridIndices.size();i += 3)
           {

               (*objfile )<<"f " << gridIndices[i]+1 << " " << gridIndices.at(i+1)+1 << " "<< gridIndices.at(i+2)+1 << std::endl;

           }


}



void TerrainFluidSimulation::render()
{
    // Resize
    int w,h;
    bool resize = false;
    glfwGetWindowSize(_window,&w,&h);
    if (w != _width)
    {
        _width = w;
        resize = true;
    }
    if (h != _height)
    {
        _height = h;
        resize = true;
    }
    if (resize)
    {
        glViewport(0,0,_width,_height);
        _cam.SetAspectRatio(float(_width)/float(_height));
    }

    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    // bind shader
    _testShader->Bind();

    auto viewMatrix = _cam.ViewMatrix();
    _testShader->SetUniform("uProjMatrix", _cam.ProjMatrix());
    _testShader->SetUniform("uViewMatrix",viewMatrix);
    _testShader->SetUniform("uViewMatrixNormal", transpose(inverse(viewMatrix)) );
    _testShader->SetUniform("uGridSize",(int)_simulationState.terrain.width());
    _testShader->SetUniform("uHardnessMode",(bool) _hardness_mode);
    // bind data
    _gridCoordBuffer.MapData(_testShader->AttributeLocation("inGridCoord"));
    _terrainHeightBuffer.MapData(_testShader->AttributeLocation("inTerrainHeight"));
    _waterHeightBuffer.MapData(_testShader->AttributeLocation("inWaterHeight"));
    _sedimentBuffer.MapData(_testShader->AttributeLocation("inSediment"));
    _simDataBuffer.MapData(_testShader->AttributeLocation("inSimData"));

    _normalBuffer.MapData(_testShader->AttributeLocation("inNormal"));

    _gridIndexBuffer.Bind();

    // render terrain
    _testShader->SetUniform("uColor", vec4(242.0/255.0,224.0/255.0,201.0/255.0,1));
    _testShader->SetUniform("uIsWater",false);
    glDrawElements(GL_TRIANGLES, _gridIndexBuffer.IndexCount(), _gridIndexBuffer.IndexType(),0);

    // unbind shader
    _testShader->UnBind();


    if(_debug_mode)
    {
        RenderDebugTool();
    }



    // finish
    glfwSwapBuffers(_window);
    glFinish();
}

void TerrainFluidSimulation::RenderDebugTool()
{
    //Arrow Data
    Grid2D<vec4> arrowCoords;
    std::vector<uint> arrowIndices;
    //y 150 x 10
    //y 153 x 190
    //int y = 153;
    //int x = 190;
    //int y = 150;
    //int x = 10;
    int y = 165;
    int x = 160;
    glm::vec3 normal(_simulation.uVel(y,x),_simulation.vVel(y,x),_simulation.zVel(y,x));
    glm::vec3 empty_vector(0,0,0);
    if(glm::any( glm::isnan(normal)) )
    {
        normal = glm::vec3(0,0,1);
    }
    else
    {

        if(glm::all(glm::equal(normal,empty_vector)))
        {
            normal = glm::vec3(0,0,1);
        }
        else
        {
            glm::normalize(normal);
        }
    }

  float y1 = (float) 2*((float)y/ 100)-3;
  float x1 = (float) 2*((float)x/ 100)-3;

    glm::vec3 point(x1,y1,(float)((_simulationState.terrain(y,x))*2)/100  );
    ArrowData(arrowCoords,arrowIndices,point,normal);

    _arrowIndexBuffer.SetData(arrowIndices);
    _arrowCoordBuffer.SetData(arrowCoords);
     // bind the arrow shader
    _arrowShader->Bind();

    auto viewMatrix = _cam.ViewMatrix();
    _arrowShader->SetUniform("uProjMatrix", _cam.ProjMatrix());
    _arrowShader->SetUniform("uViewMatrix",viewMatrix);
    _arrowShader->SetUniform("uViewMatrixNormal", transpose(inverse(viewMatrix)) );

    // bind data
    _arrowCoordBuffer.MapData((_arrowShader->AttributeLocation("point")));

    _arrowIndexBuffer.Bind();

    glDrawElements(GL_TRIANGLES, _arrowIndexBuffer.IndexCount(), _arrowIndexBuffer.IndexType(),0);

    // unbind shader
    _arrowShader->UnBind();


}



void TerrainFluidSimulation::ArrowData( Grid2D<vec4>& arrowCoords, std::vector<uint>& arrowIndices,glm::vec3 startingPoint,glm::vec3 normal)
{
    glm::vec4 offset(startingPoint[0],startingPoint[1],startingPoint[2],1);
    glm::vec3 up(0,0,1);
    glm::quat rot_quat = glm::orientation(normal,up);

    float arrow_length=0.7;
    float dimX = 0.05;
    float dimY = 0.05;
    arrowCoords.resize(4,2);

    //generate vertices
    arrowCoords(0) = glm::vec4( -dimX,  -dimY,    arrow_length,1);
    arrowCoords(1) = glm::vec4( dimX,  -dimY,    arrow_length,1);
    arrowCoords(2) = glm::vec4( dimX,   dimY,    arrow_length,1);
    arrowCoords(3) = glm::vec4( -dimX,   dimY,    arrow_length,1);

    arrowCoords(4) = glm::vec4( -dimX, -dimY,   0,1);
    arrowCoords(5) = glm::vec4(  dimX, -dimY,   0,1);
    arrowCoords(6) = glm::vec4(  dimX,  dimY,   0,1);
    arrowCoords(7) = glm::vec4( -dimX,   dimY,  0,1);


    for(uint i = 0; i < arrowCoords.size();i++)
    {
        arrowCoords(i) =    rot_quat * arrowCoords(i) + offset;
    }
    arrowIndices.clear();
arrowIndices.reserve(6*2*3);
//front
arrowIndices.push_back(0);
arrowIndices.push_back(1);
arrowIndices.push_back(2);
arrowIndices.push_back(2);
arrowIndices.push_back(3);
arrowIndices.push_back(0);


// right
arrowIndices.push_back(1);
arrowIndices.push_back(5);
arrowIndices.push_back(6);
arrowIndices.push_back(6);
arrowIndices.push_back(2);
arrowIndices.push_back(1);


// back
arrowIndices.push_back(7);
arrowIndices.push_back(6);
arrowIndices.push_back(5);
arrowIndices.push_back(5);
arrowIndices.push_back(4);
arrowIndices.push_back(7);

// left
arrowIndices.push_back(4);
arrowIndices.push_back(0);
arrowIndices.push_back(3);
arrowIndices.push_back(3);
arrowIndices.push_back(7);
arrowIndices.push_back(4);

//bottom
arrowIndices.push_back(4);
arrowIndices.push_back(5);
arrowIndices.push_back(1);
arrowIndices.push_back(1);
arrowIndices.push_back(0);
arrowIndices.push_back(4);

//top
arrowIndices.push_back(3);
arrowIndices.push_back(2);
arrowIndices.push_back(6);
arrowIndices.push_back(6);
arrowIndices.push_back(7);
arrowIndices.push_back(3);

}



void TerrainFluidSimulation::init()
{
    // Settings
#if defined(__APPLE__) || defined(__MACH__)
    std::string resourcePath = osx_GetBundleResourcesPath()+"/";
#else
    std::string resourcePath("Resources/");
#endif

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Create some data
    Grid2D<vec2> gridCoords;
    std::vector<uint> gridIndices;


    uint dimX = _simulationState.terrain.width();
    uint dimY = _simulationState.terrain.height();

    Grid2DHelper::MakeGridIndices(gridIndices,dimX,dimY);
    Grid2DHelper::MakeUniformGrid(gridCoords,dimX,dimY);


    // Send data to the GPU
    _gridIndexBuffer.SetData(gridIndices);
    _gridCoordBuffer.SetData(gridCoords);


    _terrainHeightBuffer.SetData(_simulationState.terrain);
    _waterHeightBuffer.SetData(_simulationState.water);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _simDataBuffer.SetData(_simulationState.simData);

    // Load and configure shaders
    _testShader = _shaderManager.LoadShader(resourcePath+"lambert_v.glsl",resourcePath+"lambert_f.glsl");
    _testShader->MapAttribute("inGridCoord",0);
    _testShader->MapAttribute("inTerrainHeight",1);
    _testShader->MapAttribute("inWaterHeight",2);
    _testShader->MapAttribute("inSediment",3);
    _testShader->MapAttribute("inSimData",4);

    _testShader->MapAttribute("inNormal",7);



    //arrow shader

    _arrowShader = _shaderManager.LoadShader(resourcePath+"lambert_v_arrow.glsl",resourcePath+"lambert_f_arrow.glsl");
    _arrowShader->MapAttribute("point",0);

    _cam.TranslateGlobal(vec3(0.0f,0.2,2));

    // OpenGL Settings
    glClearColor(0.4f,0.4f,0.4f,0.0f);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
//    glFrontFace(GL_CW); // clockwise
//    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    
    _terrainHeightBuffer.SetData(_simulationState.terrain);
    _waterHeightBuffer.SetData(_simulationState.water);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _simDataBuffer.SetData(_simulationState.simData);
    _normalBuffer.SetData(_simulationState.surfaceNormals);
    _inPause = false;
}


