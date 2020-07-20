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
#include <random>
typedef std::mt19937 RANDOM;
using namespace glm;
using namespace Graphics;

RANDOM rnd1;
TerrainFluidSimulation::TerrainFluidSimulation(GLFWwindow* window, uint dim)
    : _simulationState(dim,dim),
      _simulation(_simulationState),
      _rain(false),
      _air(false),
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
         ulong time;
        if(! _inPause)
        {
           time = std::chrono::duration_cast<seconds>(accumulator).count();
            //std::cout<<"time "<<time<<std::endl;
            // physics simulation
            updatePhysics(dt,time);
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
          //  accumulator = high_resolution_clock::duration(0);
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
        render(time);
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

    if (glfwGetKey(_window,'B')) _air = true;
    if (glfwGetKey(_window,'N')) _air = false;

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

float TerrainFluidSimulation::getSedimentHistory(int y, int x) {
    return (float)sedimentation_history(glm::clamp(y,0,(int) sedimentation_history.height()-1),glm::clamp(x,0,(int) sedimentation_history.width()-1)).back();
}

float TerrainFluidSimulation::getSedimentHistorySize(int y, int x) {
    return sedimentation_history(glm::clamp(y,0,(int) sedimentation_history.height()-1),glm::clamp(x,0,(int) sedimentation_history.width()-1)).size();
}

void TerrainFluidSimulation::updatePositions(){
std::cout<<"uppi"<<std::endl;
    for (uint y=0; y<_simulation.water.height(); y++)
   {
       for (uint x=0; x<_simulation.water.width(); x++)
       {
     AveragePositionSedimentationPoints(y,x);
    }
   }
       }

void TerrainFluidSimulation::updateSedimentationHistory(ulong time){
     float treshold = .01f;  //0.01//0.1f
     int counter_treshold = 10; //10 //3

//     float hysteresis = 0.0f;
     int stack_id;

     stack_id = floorf(time/20);

     for (uint y=0; y<_simulation.water.height(); y++)
    {
        for (uint x=0; x<_simulation.water.width(); x++)
        {

            //int temp_sed = _simulation.sedimented_material(y,x);
            int temp_sed = _simulation.tmp_sediment_material(y,x);

            std::vector<int>& local_sedimentation_history = sedimentation_history(y,x);
            //if(_simulation.sedimented_terrain(y,x)>0){

    bool sedimentation = false;
    bool removal = false;
              //  if(local_sedimentation_history.back() != temp_sed && temp_sed!=0 )
              //  {
                    glm::vec3 new_sedimentation_point(y,x,_simulation.terrain(y,x));
                  //  glm::vec3 old_sedimentation_point = initial_sedimentation_points(y,x).back();

                    if(local_sedimentation_history.size() == 1 && temp_sed!=0){
                        initial_sedimentation_points(y,x).push_back(new_sedimentation_point);
                        sedimentation_history(y,x).push_back(temp_sed);
                    //    sedimentation_history(y,x).push_back(temp_sed);
                    //    sedimentation_stack_id(y,x).push_back(stack_id);
                        sedimentation_stack_id(y,x).push_back(stack_id);
                        sedimentation = true;                      
                    }else{

                        if(  new_sedimentation_point[2] > initial_sedimentation_points(y,x).back()[2] + treshold && temp_sed!=0)    {
                            if(local_sedimentation_history.back() != temp_sed ){
                                if(sed_counter(y,x).second == temp_sed){
                                    sed_counter(y,x).first++;
                                }else{
                                    sed_counter(y,x).second = temp_sed;
                                    sed_counter(y,x).first = 1;
                                }
                            if(sed_counter(y,x).first>=counter_treshold){
                                initial_sedimentation_points(y,x).push_back(new_sedimentation_point);
                                sedimentation_history(y,x).push_back(temp_sed);
                                sedimentation_stack_id(y,x).push_back(stack_id);
                                sedimentation = true;
                                sed_counter(y,x).first = 0;
                                sed_counter(y,x).second = 0;
                                }
                            }
                            else{
                                sed_counter(y,x).first--;
                            }
                        }else{
                            sed_counter(y,x).first--;
                        }

                    }
                    int temp_data = sedimentation_history(y,x).back();
              //  treshold = 0.0f;
                if(initial_sedimentation_points(y,x).size()>=1 )   {
                        glm::vec3 sed_point = initial_sedimentation_points(y,x).back();
                        float h = _simulation.terrain(y,x);
                        if(h + treshold   < sed_point[2]  )    {
                            //_simulation.tmp_sediment_material(y,x) = sedimentation_history(y,x).back();

                            if(initial_sedimentation_points(y,x).size()==1){
                                initial_sedimentation_points(y,x).pop_back();
                                initial_sedimentation_points(y,x).push_back(glm::vec3(y,x,_simulation.terrain(y,x)));
                               // tmp_sedimentation_history(y,x) = sedimentation_history(y,x).back();
                            }else   {
                                initial_sedimentation_points(y,x).pop_back();
                                sedimentation_history(y,x).pop_back();
                                sedimentation_stack_id(y,x).pop_back();/*
                                for(int i=0;i<initial_sedimentation_points(y,x).size();i++){
                                    if( h + treshold < initial_sedimentation_points(y,x).back()[2] ){
                                        sedimentation_history(y,x).pop_back();
                                        sedimentation_stack_id(y,x).pop_back();
                                        initial_sedimentation_points(y,x).pop_back();
                                    }else{

                                        break;
                                    }
                                }


                                initial_sedimentation_points(y,x).push_back(glm::vec3(y,x,h));*/
                               //tmp_sedimentation_history(y,x) = sedimentation_history(y,x).back();
                            }
                            removal = true;
                            sed_counter(y,x).first = 0;
                            sed_counter(y,x).second = 0;
                        }
                }

        if(sed_counter(y,x).first<0)    {
                         sed_counter(y,x).first = 0;
                       //  sed_counter(y,x).second = 0;
        }
//        if(_simulation.sedimented_terrain(y,x)<0)
             _simulation.tmp_sediment_material(y,x) = temp_data;
        sed_color(y,x) =sedimentation_history(y,x).back();
     //   sed_color(y,x) =_simulation.tmp_sediment_material(y,x);
        if(sedimentation && removal){
            std::cout <<" sedimentation and erosion at saame time?"<<std::endl;
        }

        }
    }


}

void TerrainFluidSimulation::updatePhysics(double dt,ulong time)
{
    // Run simulation
    _simulation.update(time,dt,_rain,_flood,_air);

    updateSedimentationHistory(time);
    //if(time%10<3)
    //    updatePositions();
    // Copy data to GPU
    _terrainHeightBuffer.SetData(_simulationState.terrain);
    _waterHeightBuffer.SetData(_simulationState.water);
    _airHeightBuffer.SetData(_simulationState.air);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _sedimentedTerrainBuffer.SetData(_simulationState.sedimented_terrain);
    _sedimentedTerrainColorBuffer.SetData(sed_color);
   // _simDataBuffer.SetData(_simulationState.simData);
   // _simDataBuffer_2.SetData(_simulationState.simData_2);
    _normalBuffer.SetData(_simulationState.surfaceNormals);

}

std::vector<int> TerrainFluidSimulation::CreateMockUpData(int y,int x,int num_levels,int num_materials){
    std::vector<int> list_material_id ;

    glm::vec3 initial_point = initial_sedimentation_points(y,x)[1];
    glm::vec3 actual_point = glm::vec3(y,x,_simulation.terrain(y,x));
    int j = 1;
    std::uniform_int_distribution<ushort> rndInt(0,100);
    for(int i = 0;i<num_levels;i++){
        j = j%(num_materials);
        list_material_id.push_back(j);
        ushort res = rndInt(rnd1);
        if(res<15)
            j--;
        else
            if(res>45)
                j++;
        j = std::max(1,j);

    }
    return list_material_id;
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

void TerrainFluidSimulation::AveragePositionSedimentationPoints(int y,int x){
    //std::vector<glm::vec3> actual_point = initial_sedimentation_points(y,x);
    uint l = initial_sedimentation_points(y,x).size();
    std::vector<float> averaged_heights;
 //   if(l>1)
 //       l--;
  //  l = 6;
    float z = _simulation.terrain(y,x);
    int radius = 1;
    for(uint k = 0;k<l;k++){

        float target_height;
        if(k<l-1)
            target_height = initial_sedimentation_points(y,x)[k+1][2];
        else
            target_height = z;
        int counter = 0;
        float total_height = 0;
        int target_material_id = sedimentation_history(y,x)[k];
        if(k<initial_sedimentation_points(y,x).size()){
            for(int i= y-radius;i<y+radius;i++){
                for(int j= x-radius;j<x+radius;j++){
                    j = glm::clamp(j,0,(int) sedimentation_history.height()-1);
                    i = glm::clamp(i,0,(int) sedimentation_history.height()-1);
                    std::vector<glm::vec3> temp_list_of_points = initial_sedimentation_points(i,j);
                    if(temp_list_of_points.size()==0)
                        continue;
                    std::vector<int> temp_list_of_materials = sedimentation_history(i,j);
                    bool found = false;
                    float min_dist = INFINITY;
                    int index = 0;
                    float closest_height;
                    for(int f = 0;f<temp_list_of_materials.size();f++){
                        float height = temp_list_of_points[f][2];
                        float dist = std::abs(height-target_height);
                        if(dist<min_dist){
                            min_dist = dist;
                            closest_height = height;
                            index = f;
                        }
                        if(temp_list_of_materials[f]==target_material_id  ){
                            glm::vec3 pos(y,x,0) ;
                            found = true;
                            if(f<temp_list_of_points.size()-1)
                            {
                                pos= temp_list_of_points[f+1];

                            }else{
                                pos[2] = _simulation.terrain(i,j);
                            }
                            total_height+=pos[2];
                            counter++;
                        }

                    }
                    if(std::abs(_simulation.terrain(i,j)-target_height)<min_dist){
                            closest_height = _simulation.terrain(i,j);

                }

                    if(!found){
                        total_height+= closest_height;
                         counter++;
                    }
                    /*
                    if(k<temp_list_of_materials.size()){
                        if(temp_list_of_materials[k]==target_material_id){
                            glm::vec3 pos = temp_list_of_points[k];
                            total_height+=pos[2];
                            counter++;
                        }

                    }*/
                }
            }
        }
      if(counter!=0){
            float avg = (float) total_height / (float) counter;
           // std::cout<<"avg h "<< avg<<std::endl;
            if(k<l-1)
                averaged_heights.push_back(avg);
//                initial_sedimentation_points(y,x)[k+1][2] = avg;

      }else{
          averaged_heights.push_back(-INFINITY);
      }
    }
}

void TerrainFluidSimulation::SmoothData(std::vector<int>& local_sediments_history,
                                        std::vector<int>& local_sediments_stack_ids,
                                        std::vector<glm::vec3>& local_sediments_points){
float treshold = 0.5;


std::vector<glm::vec3>::iterator it_points = local_sediments_points.begin();
std::vector<int>::iterator it_stack_ids = local_sediments_stack_ids.begin();
for(std::vector<int>::iterator it = local_sediments_history.begin(); it != local_sediments_history.end(); ++it,++it_points,++it_stack_ids){
    //if(local_sediments_history[i]==local_sediments_history[i+2]){
std::vector<int>::iterator two_index_ahead = it+2;
if(two_index_ahead < local_sediments_history.end())
    if(*it==*two_index_ahead){
        std::vector<int>::iterator three_index_ahead = two_index_ahead+1;
        if(three_index_ahead<local_sediments_history.end())
        //if(*(it+3)<local_sediments_history.size()-1)
        {
                //if(  local_sediments_history[i+1]==local_sediments_history[i+3]){
            if(*three_index_ahead == *(it+1)){
//                    float widht_first_stack =  local_sediments_points[i+1][2] - local_sediments_points[i][2];
//                    float widht_second_stack = local_sediments_points[i+2][2] - local_sediments_points[i+1][2];

                        float widht_first_stack =  (*(it_points+1))[2] - (*(it_points))[2];
                        float widht_second_stack = (*(it_points+2))[2] - (*(it_points+1))[2];

                    if(widht_first_stack<widht_second_stack){
                        //    local_sediments_history.erase(local_sediments_history.begin()+i+1,local_sediments_history.begin()+i+2);
                        //    local_sediments_points.erase(local_sediments_points.begin()+i+1  ,local_sediments_points.begin()+i+2);
                        local_sediments_history.erase((it+1),two_index_ahead);
                        local_sediments_stack_ids.erase(it_stack_ids+1,it_stack_ids+2);
                        local_sediments_points.erase((it_points+1)  ,(it_points+2));
                    }else{
                            //local_sediments_history.erase(local_sediments_history.begin()+i+2,local_sediments_history.begin()+i+3);
                            //local_sediments_points.erase(local_sediments_points.begin()+i+2  ,local_sediments_points.begin()+i+3);
                        local_sediments_history.erase((two_index_ahead),three_index_ahead);
                        local_sediments_stack_ids.erase((it_stack_ids+2),(it_stack_ids+3));
                        local_sediments_points.erase((it_points+2)  ,(it_points+3));
                    }
                    std::cout<<"happen_1"<<std::endl;

                }else{
                    //local_sediments_history.erase(local_sediments_history.begin()+i+1,local_sediments_history.begin()+i+2);
                    //local_sediments_points.erase(local_sediments_points.begin()+i+1  ,local_sediments_points.begin()+i+2);
                    local_sediments_history.erase((it+1),two_index_ahead);
                    local_sediments_points.erase((it_points+1)  ,(it_points+2));
                    std::cout<<"happen_1"<<std::endl;
                }
            }
            else{
                local_sediments_history.erase((it+1),two_index_ahead);
                local_sediments_stack_ids.erase(it_stack_ids+1,it_stack_ids+2);
                local_sediments_points.erase((it_points+1)  ,(it_points+2));
                std::cout<<"happen_1"<<std::endl;
            }
        }


     /*   if(next_point[2] - actual_point[2]< treshold+0.1 && local_sediments_history[i]==local_sediments_history[i+2]){
            local_sediments_history.erase(local_sediments_history.begin()+i,local_sediments_history.begin()+i+2);
            local_sediments_points.erase(local_sediments_points.begin()+i,local_sediments_points.begin()+i+2);
        }*/
    }
//it_points = local_sediments_points.begin();
//it_stack_ids = local_sediments_stack_ids.begin();
std::vector<int>::iterator it = local_sediments_history.begin();
//local_sediments_history.erase(it);
//local_sediments_stack_ids.erase(it_stack_ids);
//local_sediments_points.erase(it_points);

/*for( it ; it != local_sediments_history.end(); ++it,++it_points,++it_stack_ids){
    std::vector<int>::iterator next_index = it+1;
    if(next_index<local_sediments_history.end()){
        if(*it == *next_index){
            std::cout<<"happen_2"<<*it<<std::endl;
            local_sediments_history.erase(it);
            local_sediments_stack_ids.erase(it_stack_ids);
            local_sediments_points.erase(it_points);
        }
    }
}*/


}

void TerrainFluidSimulation:: SaveSimulationData(std::fstream *datafile)
{
    bool useMockUpData = false;
    int num_materials = 5;
    int num_sedimentation_levels = 10;
    int counter = 0;
    int temp = 0;
    std::string data_string;
    std::cout<<"Writing to some file"<<std::endl;

    for (uint y=0; y < _simulationState.water.height(); y++)
    {
        for (uint x=0; x < _simulationState.water.width(); x++)
        {
            temp++;
            //(*datafile )<< _simulationState.simData(x,y);
            //(*datafile )<<" "<< _simulationState.vegetation(x,y);
            AveragePositionSedimentationPoints(y,x);
            float z = _simulation.terrain(y,x);
            (*datafile)<< "hardness "<< GetTerrainCapacity(x,y,z,_simulation.noise_sediment_frequency,_simulation._stratified_layer_width);

            //vec3 flowNormal (_simulation.uVel(x,y),_simulation.vVel(x,y),_simulation.zVel(x,y));
            vec3 flowNormal(NAN,NAN,NAN);

            if(dot(_simulation.flowNormal(y,x),_simulation.flowNormal(y,x))> 0.001)//  &&  _simulationState.simData_2(x,y)>0)
            {
                flowNormal = _simulation.flowNormal(y,x);//_simulation.count(x,y));
                flowNormal = normalize(flowNormal);
            }

            (*datafile )<<" flow_normal v "<< flowNormal[0]  << " "<< flowNormal[1]<< " "<< flowNormal[2];

            glm::vec3 point = initial_sedimentation_points(y,x)[0];
            std::vector<int> local_sediments_history = sedimentation_history(y,x) ;
            std::vector<glm::vec3> local_sediments_points = initial_sedimentation_points(y,x);
            if(useMockUpData){
                std::vector<int> mock_up =  CreateMockUpData(y,x,num_sedimentation_levels,num_materials);
                (*datafile)<< " sediment_value l "<< mock_up.size()<< " ";
                for(uint i =0;i<mock_up.size();i++){
                    (*datafile)<< mock_up[i]<<" ";
                }
                (*datafile)<< " initial_sedimentation_point v " << point[0]<< " "<< point[1]<< " "<<point[2]<< " ";
            }else{
                if(local_sediments_history.size()>1 && z-point[2]>0){
                    glm::vec3 actual_point(y,x,_simulation.terrain(x,y));
                    std::vector<int> local_stack_id = sedimentation_stack_id(y,x);
                    //glm::vec3 dist = actual_point-point;
                    //if(dist[2] > 0){
                    point = initial_sedimentation_points(y,x)[1];
             //       SmoothData(local_sediments_history,local_stack_id,local_sediments_points);

                    (*datafile)<< " sediment_value l "<< local_sediments_history.size()-1<< " ";
                    for(uint i = 1 ;i<local_sediments_history.size();i++){
                        float entry = local_sediments_history[i];
                        if(entry != 0){
                             (*datafile)<< entry<< " ";
                           }
                    }uint i = 0;

                    (*datafile)<< " material_stacks_id l "<<local_stack_id.size()-1<<" ";
                   for(uint i = 1 ;i<local_stack_id.size();i++)
                   {
                       int entry = local_stack_id[i];
                       (*datafile)<<  entry << " ";
                   }

                    point = local_sediments_points[0][2]<local_sediments_points[1][2] ? local_sediments_points[0] : local_sediments_points[1];

                    (*datafile)<< " material_stacks_height l "<<local_sediments_points.size()-1<<" ";
                    (*datafile )<< " 0 ";
                    for(uint i = 2 ;i<local_sediments_points.size();i++)
                   {
                       glm::vec3 entry = local_sediments_points[i];
                       if(entry[2]>=point[2]){
                           (*datafile)<<  entry[2] - point[2] << " ";
                       }
                   }



                    //(*datafile)<< z - point[2]<<" ";
                    (*datafile)<< " initial_sedimentation_point v " << point[0]<< " "<< point[1]<< " "<<point[2]<< " " ;
                    (*datafile)<<" actual_point v "<< y << " "<< x << " " <<   z ;
                    (*datafile)<< " sed_level "<<_simulation.sedimented_terrain(y,x);
                    (*datafile)<< " diff "<< z-point[2];
                   float pressure = ((float)_simulation.air_total_pressure(y,x)) /(float)_simulation.air_num_samples(y,x);
                     (*datafile)<<" pressure "<<pressure;
                }
            }



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
                   z = _simulationState.terrain(y,x);

                   (*objfile )<<"v " << y << " " << x << " "<< z << std::endl;
               }
           }
           std::vector<uint> gridIndices;
           //Grid2DHelper::MakeGridIndicesClockWiseOrder(gridIndices,_simulationState.terrain.width(),_simulationState.terrain.height());
           Grid2DHelper::MakeGridIndices(gridIndices,_simulationState.terrain.width(),_simulationState.terrain.height());

           std::cout<<"gridIndices.size() "<< gridIndices.size()<<std::endl;
           uint i = 0;
           for (i = 0;i<gridIndices.size();i += 3)
           {

               (*objfile )<<"f " << gridIndices[i]+1 << " " << gridIndices.at(i+1)+1 << " "<< gridIndices.at(i+2)+1 << std::endl;

           }


}



void TerrainFluidSimulation::render(ulong time)
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
    _airHeightBuffer.MapData(_testShader->AttributeLocation("inAirHeight"));
    _sedimentBuffer.MapData(_testShader->AttributeLocation("inSediment"));
    _sedimentedTerrainBuffer.MapData(_testShader->AttributeLocation("inSedimentedTerrain"));
    _sedimentedTerrainColorBuffer.MapData(_testShader->AttributeLocation("inSedimentedTerrainColor"));
  //  _simDataBuffer.MapData(_testShader->AttributeLocation("inSimData"));
  //  _simDataBuffer_2.MapData(_testShader->AttributeLocation("inSimData_2"));
    //vec4 sedimentedTerrainColor(1,0,0,1);
    //if(time>10)
    //    sedimentedTerrainColor = vec4(0,1,0,1);
    //_testShader->SetUniform("usedimentedTerrainColor", sedimentedTerrainColor);


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
    //int y = 143;
    //int x = 45;
    int y = 132;
    int x = 95;
    glm::vec3 normal = _simulation.flowNormal(y,x);
    //glm::vec3 normal(_simulation.uVel_air(y,x),_simulation.vVel_air(y,x),_simulation.zVel_air(y,x));
    glm::vec3 empty_vector(0,0,0);
    if(glm::any( glm::isnan(normal)) )    {
        normal = normalize(glm::vec3(0,1,1));
    }
    else
    {

        if(glm::all(glm::equal(normal,empty_vector)))        {
            normal = glm::vec3(0,0,1);
        }
        else        {
            normal=  glm::normalize(normal);
        }
    }
    if(glm::any( glm::isnan(normal)) )    {
        normal = normalize(glm::vec3(0,1,1));
    }
    std::cout<<"normal "<< normal[0] <<" "<< normal[1] <<" "<< normal[2]<<std::endl;
    //vec3 wind_direction(0.1,0.5,0);
    //normalize(wind_direction);
  //  normal =  glm::vec3(_simulationState.windDirection[0],_simulationState.windDirection[1],0);
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
    _airHeightBuffer.SetData(_simulationState.air);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _sedimentedTerrainBuffer.SetData(_simulationState.sedimented_terrain);
    _sedimentedTerrainColorBuffer.SetData(sed_color);
    _simDataBuffer.SetData(_simulationState.simData);
    _simDataBuffer_2.SetData(_simulationState.simData_2);

    // Load and configure shaders
    _testShader = _shaderManager.LoadShader(resourcePath+"lambert_v.glsl",resourcePath+"lambert_f.glsl");
    _testShader->MapAttribute("inGridCoord",0);
    _testShader->MapAttribute("inTerrainHeight",1);
    _testShader->MapAttribute("inWaterHeight",2);
    _testShader->MapAttribute("inAirHeight",8);
    _testShader->MapAttribute("inSediment",3);
    _testShader->MapAttribute("inSedimentedTerrain",5);
    _testShader->MapAttribute("inSedimentedTerrainColor",9);

    _testShader->MapAttribute("inSimData",4);
    _testShader->MapAttribute("inSimData_2",6);
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
     _airHeightBuffer.SetData(_simulationState.air);
    _sedimentBuffer.SetData(_simulationState.suspendedSediment);
    _sedimentedTerrainBuffer.SetData(_simulationState.sedimented_terrain);
    _sedimentedTerrainColorBuffer.SetData(sed_color);
    _simDataBuffer.SetData(_simulationState.simData);
    _simDataBuffer_2.SetData(_simulationState.simData_2);
    _normalBuffer.SetData(_simulationState.surfaceNormals);
    _inPause = false;
sedimentation_history = Grid2D<std::vector<int>>(_simulation.water.width(),_simulation.water.height());
sedimentation_stack_id = Grid2D<std::vector<int>>(_simulation.water.width(),_simulation.water.height());
tmp_sedimentation_history = Grid2D<int>(_simulation.water.width(),_simulation.water.height());
sed_counter = Grid2D<std::pair<int,int>>(_simulation.water.width(),_simulation.water.height());
sed_color = Grid2D<float>(_simulation.water.width(),_simulation.water.height());
initial_sedimentation_points = Grid2D<std::vector<glm::vec3>>(_simulation.water.width(),_simulation.water.height());
    for (uint y=0; y<_simulation.water.height(); y++)
    {
        for (uint x=0; x<_simulation.water.width(); x++)
        {

            float z = _simulation.terrain(y,x);
            int mat = 1;
                           int mat_id = 0;
                           std::vector<glm::vec3> points;
                           std::vector<int> v;
                              std::vector<int> ids;


                           for(float i=z;i>-20;i-=4){
                               points.push_back(glm::vec3(x,y,i));
                               v.push_back(mat);
                               ids.push_back(mat_id);
                               mat++;
                               mat_id++;
                               if(mat>8)
                                   mat = 1;
                           }
                           std::reverse(v.begin(),v.end());
                           std::reverse(ids.begin(),ids.end());
                           std::reverse(points.begin(),points.end());
                           sedimentation_history(y,x) = v;
                           sed_counter(y,x) = std::make_pair(0,0);
                           sedimentation_stack_id(y,x) = ids;
                           initial_sedimentation_points(y,x) =points;
                            //(1,glm::vec3(INFINITY,INFINITY,INFINITY))
                           tmp_sedimentation_history(y,x) = v.back();;

                //sed_counter(y,x) = std::make_pair(0,0);
               // tmp_sedimentation_history(y,x) = 0;
                //sedimentation_history(y,x) = v;
                //sedimentation_stack_id(y,x) = v;
                //initial_sedimentation_points(y,x) = std::vector<glm::vec3> (1,glm::vec3(INFINITY,INFINITY,INFINITY));


        }
    }
    std::cout<<"shader loaded"<<std::endl;
}


