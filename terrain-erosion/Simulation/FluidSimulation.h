/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef SIMULATION_H
#define SIMULATION_H

#include "platform_includes.h"
#include "Grid2D.h"
#include "Math/PerlinNoise.h"
#include "SimulationState.h"
#include <map>
#include <algorithm>
using namespace glm;




namespace Simulation {

class FluidSimulation
{

public:
    FluidSimulation(SimulationState& state);
    ~FluidSimulation();

    SimulationState& state;
    Grid2D<float>& water;
    Grid2D<float>& air;
    Grid2D<ulong>& air_total_pressure;
    Grid2D<ulong>& air_num_samples;
    Grid2D<float>& terrain;
    Grid2D<float>& sediment;
    Grid2D<float> counter_from_last_time_water_passed;
    Grid2D<float> counter_times_water_was_still;
    Grid2D<float> tmpSediment;
    Grid2D<float> tmp_sediment_material_2;

    Grid2D<float> tmp_sediment_material;
    Grid2D<float>& sedimented_terrain;
    Grid2D<float>& sedimented_material;
    Grid2D<glm::vec4>& sedimented_terrain_color;
    Grid2D<float> uVel;
    Grid2D<float> vVel;
    Grid2D<float> uVel_air;
    Grid2D<float> vVel_air;
    Grid2D<float> zVel_air;
    Grid2D<glm::vec3> flowNormal;
    Grid2D<float> count;
    Grid2D<float> lFlux_air;
    Grid2D<float> rFlux_air;
    Grid2D<float> tFlux_air;
    Grid2D<float> bFlux_air;
    Grid2D<float> lFlux;
    Grid2D<float> rFlux;
    Grid2D<float> tFlux;
    Grid2D<float> bFlux;
    float noise_sediment_frequency;
    int sediment_layers[7];
    int sediment_materials[2];
    glm::vec2 rainPos;
    glm::vec2 _windDirection;
    // lX and lY have to decrease if we increse the gridsize
    const float lX;
    const float lY;
    const float gravity;
    const float _stratified_layer_width = 0.001;

    void update(ulong time, double dt, bool makeRain=true, bool flood=false, bool wind=false);
    void simulateFlow(double dt);
    void simulateErosion(double dt, ulong time);
    void simulateSedimentTransportation(double dt, ulong time);
    void simulateEvaporation(double dt);
    float sampleTerrain(int x,int y );
    void makeRain(double dt,ulong time);
    void makeFlood(double dt, ulong time);
    void makeWind(double dt);
    void makeRiver(double dt, ulong time);
    void simulateWind(double dt);
    void EraseWater();
    void addRainDrop(const vec2& pos, int rad, float amount);
    void addWindParticle(const vec2& pos, int rad, float amount);

    void smoothTerrain();

    void computeSurfaceNormals();

    float GetTerrainCapacity(float x,float y,float z ,float frequency,int* layers );
    // flux access (takes care of boundaries)
    inline float getRFlux(int y, int x);
    inline float getLFlux(int y, int x);
    inline float getBFlux(int y, int x);
    inline float getTFlux(int y, int x);

    float getSedimentMaterial(int y, int x);

    void EraseAll();
    void smoothSediment();
    inline float getRFlux_air(int y, int x);
    inline float getLFlux_air(int y, int x);
    inline float getBFlux_air(int y, int x);
    inline float getTFlux_air(int y, int x);

    // terrain access
    //inline float getTerrain(int y, int x);
    float getTerrain(int y, int x);
    // water access
    inline float getWater(int y, int x);
inline float getAir(int y, int x);

};

}
#endif // SIMULATION_H
