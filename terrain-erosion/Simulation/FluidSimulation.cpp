/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#include "FluidSimulation.h"
#include <algorithm>
#include <random>
typedef std::mt19937 RANDOM;  // the Mersenne Twister with a popular choice of parameters

#if defined(__GNUG__)
#include "Math/MathUtil.h"
#else
#include "MathUtil.h"
#endif

#if defined(__APPLE__) || defined(__MACH__)
#include <dispatch/dispatch.h>
dispatch_queue_t gcdq = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
#endif

using namespace Simulation;
using namespace glm;
using namespace std;



FluidSimulation::FluidSimulation(SimulationState& state)
    : state(state),
      water(state.water),
      air(state.air),
      air_total_pressure(state.air_total_pressure),
      air_num_samples(state.air_counter),
      terrain(state.terrain),
      sediment(state.suspendedSediment),
      sedimented_terrain(state.sedimented_terrain),
      sedimented_terrain_color(state.sedimented_terrain_color),
      sedimented_material(state.sedimented_material),
      counter_from_last_time_water_passed(state.vegetation),
      counter_times_water_was_still(state.vegetation),
      tmpSediment(state.water.width(),state.water.height()),
      tmp_sediment_material(state.water.width(),state.water.height()),
      tmp_sediment_material_2(state.water.width(),state.water.height()),
      uVel(water.width(), water.height()),
      vVel(water.width(), water.height()),
      vVel_air(air.width(), air.height()),
      uVel_air(air.width(), air.height()),
      zVel_air(air.width(), air.height()),
      flowNormal(air.width(), air.height()),
       count(water.width(), water.height()),
      lFlux(water.width(), water.height()),
      rFlux(water.width(), water.height()),
      tFlux(water.width(), water.height()),
      bFlux(water.width(), water.height()),
      lFlux_air(air.width(), air.height()),
      rFlux_air(air.width(), air.height()),
      tFlux_air(air.width(), air.height()),
      bFlux_air(air.width(), air.height()),
      lX(1.0),
      lY(1.0),
      gravity(9.81),
      _windDirection(state.windDirection),
      _stratified_layer_width(0.7),

      noise_sediment_frequency(0.1)
{
    assert(water.height() == terrain.height() && water.width() == terrain.width());

    // Reset velocity field
    for (uint i=0; i<uVel.width(); ++i) {
        for (uint j=0; j<uVel.height(); ++j) {
            uVel(i,j) = 0;
            vVel(i,j) = 0;
            uVel_air(i,j) = 0;
            vVel_air(i,j) = 0;
            zVel_air(i,j) = 0;
            count(i,j) = 0;

            lFlux(i,j) = rFlux(i,j) = tFlux(i,j) = bFlux(i,j) = 0;
            lFlux_air(i,j) = rFlux_air(i,j) = tFlux_air(i,j) = bFlux_air(i,j) = 0;
            tmp_sediment_material(i,j) = 0;
        }
    }
    for(uint i = 0;i<air.width();i++){
        for(uint j = 0;j<air.height();j++){
            flowNormal(i,j)= vec3(0,0,0);
        }
    }
            sediment_layers[0] = 16;
            sediment_layers[1] = 30;
            sediment_layers[2] = 16;
            sediment_layers[3] = 30;
            sediment_layers[4] = 16;
            sediment_layers[5] = 30;
            sediment_layers[6] = 35;


            sediment_materials[0] = 1;
            sediment_materials[1] = 2;
           // sediment_materials[2] = 3;


}

FluidSimulation::~FluidSimulation() {
//    delete grid;
}

RANDOM rnd;

void FluidSimulation::makeWind(double dt){
    std::uniform_real_distribution<float> rndFloat(0,1);
  //  std::uniform_real_distribution<float> rndFloat(0,5);
    std::uniform_real_distribution<float> rndFloat2(-1,1);
std::uniform_int_distribution<int> rndint(0,100);
if(rndint(rnd)==100){
    float x = rndFloat(rnd);
    float y = rndFloat2(rnd);
    _windDirection[0] = x;
    _windDirection[1] = y;

}
    for(int x = 0 ; x<air.height();x++)
        air(x,0.01) = 1;
        //const vec2 pos = vec2(0.01,water.height()/2);
        //air(air.height()/2,0.01) = 5;
}


void FluidSimulation::makeRiver(double dt,ulong time)

{
   // float scaler = 0.01;
    float scaler = 0.05;
    int maxrand = 1000;
    std::uniform_int_distribution<ushort> rndInt(1,3);
  //  std::uniform_real_distribution<float> rndFloat(0,5);
    std::uniform_real_distribution<float> rndFloat(0,2);
    std::uniform_int_distribution<ushort> rndInt2(0,maxrand);

//    const vec2 pos = vec2(0.01,water.height()/2);
        const vec2 pos = vec2(water.height()*1/3-5,water.height()/2);

    int x = rndInt(rnd);
    int z = rndInt2(rnd);
    int y = rndFloat(rnd);
    addRainDrop(pos,x,dt*scaler*y);

    //simulate occasional flooding
    if(z==maxrand)
        addRainDrop(pos,x,dt*3*y);


}
void FluidSimulation::makeRain(double dt,ulong time)
{
    std::uniform_int_distribution<ushort> rndInt(1,water.width()-2);
    std::uniform_int_distribution<ushort> rndIntX(1,water.height()*1/3-2);
    std::uniform_int_distribution<ushort> rndIntY(1,water.width()-2);
std::cout<<"r"<<std::endl;
    int sediment_material = 1;
            int scaler = 2;
    if(time%(50*scaler)>scaler*10){
        //std::cout<<"new mat"<<std::endl;
        sediment_material = 2;
    }
    if(time%(50*scaler)>scaler*20){
       // std::cout<<"new mat 2"<<std::endl;
        sediment_material = 3;
    }
    if(time%(50*scaler)>scaler*30){
        //std::cout<<"new mat 4"<<std::endl;
        sediment_material = 4;
    }
    if(time%(50*scaler)>scaler*40){
        //std::cout<<"new mat 5"<<std::endl;
        sediment_material = 5;
    }
    if(time>250)
        sediment_material = 0;
    std::cout<<" "<<sediment_material<<std::endl;
    for (uint i=0; i<100; i++)
    {
        uint x = rndIntX(rnd);
        uint y = rndIntY(rnd);
//        uint x = rndInt(rnd);
//        uint y = rndInt(rnd);

//        water(y,x) += 1;
        water(y-1, x-1)  += 1.0/16.0;
        water(y-1, x)  += 1.0/16.0;
        water(y-1, x+1)  += 1.0/16.0;
        water(y, x-1)  += 1.0/16.0;
        water(y,x)  += 1.0/16.0;
        water(y, x+1)  += 1.0/16.0;
        water(y+1, x-1)  += 1.0/16.0;
        water(y+1, x)  += 1.0/16.0;
        water(y+1, x+1)  += 1.0/16.0;

        tmp_sediment_material(y-1,x-1) = sediment_material;
        tmp_sediment_material(y-1,x) = sediment_material;
        tmp_sediment_material(y-1,x+1) = sediment_material;
        tmp_sediment_material(y,x-1) = sediment_material;
        tmp_sediment_material(y,x) = sediment_material;
        tmp_sediment_material(y,x+1) = sediment_material;
        tmp_sediment_material(y+1,x-1) = sediment_material;
        tmp_sediment_material(y+1,x) = sediment_material;
        tmp_sediment_material(y+1,x+1) = sediment_material;

    }

//    std::uniform_real_distribution<float> rndFloat(0.0f,1.0f);
//    for (uint i=0; i<water.size(); i++)
//    {
//        water(i) += 0.01*rndFloat(rnd);
    //    }
}

void FluidSimulation::makeFlood(double dt,ulong time)
{
    addRainDrop(rainPos,1,dt*0.01*1);
    float y = rainPos[0];
    float x = rainPos[1];
    int sediment_material = 1;
    int scaler = 1;
if(time%(50*scaler)>scaler*10){
 std::cout<<"new mat"<<std::endl;
sediment_material = 2;
}
if(time%(50*scaler)>scaler*20){
// std::cout<<"new mat 2"<<std::endl;
sediment_material = 3;
}
if(time%(50*scaler)>scaler*30){
//std::cout<<"new mat 4"<<std::endl;
sediment_material = 4;
}
if(time%(50*scaler)>scaler*40){
//std::cout<<"new mat 5"<<std::endl;
sediment_material = 5;
}
if(time>250)
sediment_material = 0;

    tmp_sediment_material(y,x) = sediment_material;
    tmp_sediment_material(y-1,x-1) = sediment_material;
    tmp_sediment_material(y-1,x) = sediment_material;
    tmp_sediment_material(y-1,x+1) = sediment_material;
    tmp_sediment_material(y,x-1) = sediment_material;
    tmp_sediment_material(y,x) = sediment_material;
    tmp_sediment_material(y,x+1) = sediment_material;
    tmp_sediment_material(y+1,x-1) = sediment_material;
    tmp_sediment_material(y+1,x) = sediment_material;
    tmp_sediment_material(y+1,x+1) = sediment_material;


    sediment(y,x)       = 1;
    sediment(y-1,x-1)   = 1;
    sediment(y-1,x)     = 1;
    sediment(y-1,x+1)   = 1;
    sediment(y,x-1)     = 1;
    sediment(y,x)       = 1;
    sediment(y,x+1)     = 1;
    sediment(y+1,x-1)   = 1;
    sediment(y+1,x)     = 1;
    sediment(y+1,x+1)   = 1;
/*
    tmp_sediment_material(y-2,x-2) = sediment_material;
    tmp_sediment_material(y-2,x) = sediment_material;
    tmp_sediment_material(y-2,x+2) = sediment_material;
    tmp_sediment_material(y,x-2) = sediment_material;
    tmp_sediment_material(y,x) = sediment_material;
    tmp_sediment_material(y,x+2) = sediment_material;
    tmp_sediment_material(y+2,x-2) = sediment_material;
    tmp_sediment_material(y+2,x) = sediment_material;
    tmp_sediment_material(y+2,x+2) = sediment_material;

    tmp_sediment_material(y-3,x-3) = sediment_material;
    tmp_sediment_material(y-3,x) = sediment_material;
    tmp_sediment_material(y-3,x+3) = sediment_material;
    tmp_sediment_material(y,x-3) = sediment_material;
    tmp_sediment_material(y,x) = sediment_material;
    tmp_sediment_material(y,x+3) = sediment_material;
    tmp_sediment_material(y+3,x-3) = sediment_material;
    tmp_sediment_material(y+3,x) = sediment_material;
    tmp_sediment_material(y+3,x+3) = sediment_material;*/


}





void FluidSimulation::addRainDrop(const vec2 &pos, int rad, float amount)
{
    int rad2 = rad*rad;
    for (int y= -rad; y<=rad; y++)
    {
        int posY = pos.y + y;
        for (int x=-rad; x<=rad; x++)
        {
            int posX = pos.x + x;
            if (posX>=0 && posY>=0 && posX<terrain.width() && posY<terrain.height())
            {
                float d = x*x+y*y;
                if (d <= rad2)
                {
                    float a = amount * (rad2-d);
                    water(posY,posX) += a;
                }
            }
        }
    }
}

//dispatch_queue_t gcdq = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
//dispatch_apply(elements.size(), gcdq, ^(size_t idx)

void FluidSimulation::smoothTerrain()
{
    float maxD = 0.2f;
    float h;
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(terrain.width(), gcdq, ^(size_t x)
#else
    #pragma omp parallel for
    for (int x=0; x<terrain.width(); ++x)
#endif
    {
        for (int y=0; y<terrain.height(); ++y)
        {
            int counter=0;
            h = getTerrain(y,x);

            float hl = getTerrain(y,x-1);
            float hr = getTerrain(y,x+1);
            float ht = getTerrain(y+1,x);
            float hb = getTerrain(y-1,x);


            float h_mat = getSedimentMaterial(y,x);
            float hl_mat = getSedimentMaterial(y,x-1);
            float hr_mat = getSedimentMaterial(y,x+1);
            float ht_mat = getSedimentMaterial(y+1,x);
            float hb_mat = getSedimentMaterial(y-1,x);
            if(h_mat>0){
                counter++;
            }
            if(hr_mat>0){
                counter++;
            }
            if(hl_mat>0){
                counter++;
            }
            if(ht_mat>0){
                counter++;
            }
            if(hb_mat>0){
                counter++;
            }
            float dl = h - hl;
            float dr = h - hr;

            float dt = h - ht;
            float db = h - hb;

            tmpSediment(y,x) = h;
float avg_material = 0;
            if ((abs(dl) > maxD || abs(dr) > maxD) && dr*dl > 0.0f)
            {
                float avg = (h+hl+hr+ht+hb)/5;
               if(counter>0)
                    avg_material  = roundf((h_mat + hl_mat +hr_mat +ht_mat +hb_mat)/counter);
                tmpSediment(y,x) = avg;
                tmp_sediment_material_2(y,x) = avg_material;
            }
            else if ((abs(dt) > maxD || abs(db) > maxD) && dt*db > 0.0f)
            {
                if(counter>0)
                    avg_material = roundf((h_mat + hl_mat +hr_mat +ht_mat +hb_mat)/counter);
                float avg = (h+hl+hr+ht+hb)/5;
                tmpSediment(y,x) = avg;
                tmp_sediment_material_2(y,x) = avg_material;
            }
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

//sedimented_material(y,x) = tmp_sediment_material(y,x);


#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(terrain.size(), gcdq, ^(size_t i)
#else
    #pragma omp parallel for
    for (uint i=0; i<terrain.size(); ++i)
#endif
    {
        float ex_t = terrain(i);
        float tmpsed = tmpSediment(i);
        float diff = ex_t - tmpsed;
        terrain(i) = tmpsed;
        sedimented_terrain(i) -= diff;
        //if(tmp_sediment_material(i) == 0 )     {
        //     tmp_sediment_material(i) = tmp_sediment_material_2(i);
        //     sedimented_material(i) = tmp_sediment_material_2(i);
        //}
    }

#if defined(__APPLE__) || defined(__MACH__)
    );
#endif
}

void FluidSimulation::computeSurfaceNormals()
{

#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(terrain.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (int y=0; y<terrain.height(); ++y)
#endif
    {
        for (int x=0; x<terrain.width(); ++x)
        {
            float r,l,t,b;
            vec3 N;
            r = getTerrain(y,x+1) + getWater(y,x+1);
            l = getTerrain(y,x-1) + getWater(y,x-1);
            t = getTerrain(y+1,x) + getWater(y+1,x);
            b = getTerrain(y-1,x) + getWater(y-1,x);

            N = vec3(l-r, t - b, 2 );
            N = normalize(N);

            state.surfaceNormals(y,x) = N;
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif
}

void FluidSimulation::simulateWind(double dt)
{

    // Outflux computation settings
    ////////////////////////////////////////////////////////////
    float l = 1;
    float A = 0.00005;
    float treshold_time = 100;
    float still_water_treshold = 0.05;
    float river_min_speed_treshold = 0.01;//0.043;
    float river_max_speed_treshold = 1;

    float river_height_treshold = 1;
    float river_min_height_treshold = 0.001;
    const float ocean_level = 0.75;
    const float dx = lX;
    const float dy = lY;

    float wx,wy;
    glm::vec2 wd(1,0);
    //wx = wd[0];//_windDirection[0];
    //wy = wd[1];//_windDirection[1];
    wx = _windDirection[0];
    wy = _windDirection[1];
    float fluxFactor_air = dt*A*gravity/l;

    // Outflow Flux Computation with boundary conditions
    ////////////////////////////////////////////////////////////
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(uVel_air.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<uVel_air.height(); ++y)
#endif
    {
        for (uint x=0; x<uVel_air.width(); ++x)
        {
            float strenght = 3;
            float dh;                               // height difference
            float h0 = terrain(y,x)+air(y,x);     // water height at current cell
            float newFlux_air;

            // left outflow
            if (x > 0)
            {
                dh =h0 - (terrain(y,x-1)+air(y,x-1));
                newFlux_air = lFlux_air(y,x) + fluxFactor_air*(dh-wx*strenght);
                lFlux_air(y,x) = std::max(0.0f,newFlux_air);
            }
            else
            {
                lFlux_air(y,x) = 0.0f; // boundary
            }

            // right outflow
            if (x < air.width()-1) {
                dh = h0 - (terrain(y,x+1)+air(y,x+1));
                newFlux_air = rFlux_air(y,x) + fluxFactor_air*(dh+wx*strenght);
                rFlux_air(y,x) = std::max(0.0f,newFlux_air);
            }
            else
            {
                rFlux_air(y,x) = 0.0f; // boundary
            }

            // bottom outflow
            if (y > 0)
            {
                dh = h0 - (terrain(y-1,x)+air(y-1,x));
                newFlux_air = bFlux_air(y,x) + fluxFactor_air*(dh-wy*strenght);
                bFlux_air(y,x) = std::max(0.0f,newFlux_air);
            }
            else
            {
                bFlux_air(y,x) = 0.0f; // boundary
            }

            // top outflow
            if (y < air.height()-1) {
                dh = h0 - (terrain(y+1,x)+air(y+1,x));
                newFlux_air = tFlux_air(y,x) + fluxFactor_air*(dh+wy*strenght);
                tFlux_air(y,x) = std::max(0.0f,newFlux_air);
            }
            else
            {
                tFlux(y,x) = 0.0f; // boundary
            }

            // scaling
            float sumFlux_air = lFlux_air(y,x)+rFlux_air(y,x)+bFlux_air(y,x)+tFlux_air(y,x);
            float K = std::min(1.0f,float((air(y,x)*dx*dy)/(sumFlux_air*dt)));
            rFlux_air(y,x) *= K;
            lFlux_air(y,x) *= K;
            tFlux_air(y,x) *= K;
            bFlux_air(y,x) *= K;
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

    // Update water surface and velocity field
    ////////////////////////////////////////////////////////////
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(uVel_air.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (int y=0; y<uVel_air.height(); ++y)
#endif
    {
        for (int x=0; x<uVel_air.width(); ++x)
        {
            if (x > uVel_air.width()-5){
                air(x,y) = 0;
                continue;
            }
            float inFlow =  getRFlux_air(y,x-1) + getLFlux_air(y,x+1) + getTFlux_air(y-1,x) + getBFlux_air(y+1,x);
            float outFlow = getRFlux_air(y,x) +   getLFlux_air(y,x) +   getTFlux_air(y,x) +   getBFlux_air(y,x);
            float dV = dt*(inFlow-outFlow);
            float oldAir = air(y,x);
            float actualAir = air(y,x);
            //water(y,x) += dV/(dx*dy);
            //water(y,x) = std::max(water(y,x),0.0f);
            actualAir += dV/(dx*dy);
            actualAir = std::max(actualAir,0.0f);


            float meanAir = 0.5*(oldAir+ actualAir);

            if (meanAir == 0.0f)
            {
                uVel_air(y,x) = vVel_air(y,x) = zVel_air(y,x)= 0.0f;
            }
            else
            {
                uVel_air(y,x) =  0.5*(getRFlux_air(y,x-1)-getLFlux_air(y,x)-getLFlux_air(y,x+1)+getRFlux_air(y,x))/(dy*meanAir);
                vVel_air(y,x) =  0.5*(getTFlux_air(y-1,x)-getBFlux_air(y,x)-getBFlux_air(y+1,x)+getTFlux_air(y,x))/(dx*meanAir);
            }

            float uV = uVel_air(y,x);
            float vV = vVel_air(y,x);
            float zV = ( actualAir-oldAir);

         //   std::cout<<"uV "<<uV<<" vV "<<vV<<" zV "<<zV<<std::endl;


            zVel_air(y,x) = zV;

            count(y,x)++;
            //flowNormal(y,x) += vec3(uV,vV,actualAir -oldAir);
            float vel = sqrtf(uV*uV+vV*vV);
            glm::vec3 local_speed_vector = vec3(uV,vV,zV);
glm::vec3 previous_vec = flowNormal(y,x);
            if(glm::any(glm::isnan(previous_vec)))
                previous_vec = glm::vec3(0,0,0);
            //flowNormal(y,x) += local_speed_vector ;
          //  local_speed_vector =  normalize(local_speed_vector);
            for(int i = 0;i<3;i++){
                local_speed_vector[i] = floorf(local_speed_vector[i]*100);
                //std::cout<<local_speed_vector[i]<<std::endl;

            }
            if(glm::any(glm::isnan(previous_vec))){
                previous_vec = glm::vec3(0,0,0);
                std::cout<<"is NAN"<<std::endl;
            }
            flowNormal(y,x) = (previous_vec + local_speed_vector);
            //float dot_prod = glm::dot(glm::normalize(local_speed_vector),wind_direction);
            air(y,x) = actualAir;// * dot_prod;
            if(actualAir>0.01){
                air_total_pressure(y,x) += (ulong) actualAir*100;
                air_num_samples(y,x)++;
            }
        }
    }

#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

}



void FluidSimulation::simulateFlow(double dt)
{

    // Outflux computation settings
    ////////////////////////////////////////////////////////////
    float l = 1;
    float A = 0.00005;
    float treshold_time = 100;
    float still_water_treshold = 0.05;
    float river_min_speed_treshold = 0.01;//0.043;
    float river_max_speed_treshold = 1;

    float river_height_treshold = 1;
    float river_min_height_treshold = 0.001;
    const float ocean_level = 0.75;
    const float dx = lX;
    const float dy = lY;

    float fluxFactor = dt*A*gravity/l;


    // Outflow Flux Computation with boundary conditions
    ////////////////////////////////////////////////////////////
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(uVel.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<uVel.height(); ++y)
#endif
    {
        for (uint x=0; x<uVel.width(); ++x)
        {
            float dh;                               // height difference
            float h0 = terrain(y,x)+water(y,x);     // water height at current cell
            float newFlux;
            float strenght = 0.1;
            // left outflow
            if (x > 0)
            {
                dh =h0 - (terrain(y,x-1)+water(y,x-1));
                newFlux =  lFlux(y,x) + fluxFactor*(dh);
                lFlux(y,x) = std::max(0.0f,newFlux);
            }
            else
            {
                lFlux(y,x) = 0.0f; // boundary
            }

            // right outflow
            if (x < water.width()-1) {
                dh =  h0 - (terrain(y,x+1)+water(y,x+1));
                newFlux =  rFlux(y,x) + fluxFactor*(dh);
                rFlux(y,x) = std::max(0.0f,newFlux);
            }
            else
            {
                rFlux(y,x) = 0.0f; // boundary
            }

            // bottom outflow
            if (y > 0)
            {
                dh = h0 - (terrain(y-1,x)+water(y-1,x));
                newFlux = bFlux(y,x) + fluxFactor*(dh);
                bFlux(y,x) = std::max(0.0f,newFlux);
            }
            else
            {
                bFlux(y,x) = 0.0f; // boundary
            }

            // top outflow
            if (y < water.height()-1) {
                dh = h0 - (terrain(y+1,x)+water(y+1,x));
                newFlux = tFlux(y,x) + fluxFactor*(dh);
                tFlux(y,x) = std::max(0.0f,newFlux);
            }
            else
            {
                tFlux(y,x) = 0.0f; // boundary
            }

            // scaling
            float sumFlux = lFlux(y,x)+rFlux(y,x)+bFlux(y,x)+tFlux(y,x);
            float K = std::min(1.0f,float((water(y,x)*dx*dy)/(sumFlux*dt)));
            rFlux(y,x) *= (K);
            lFlux(y,x) *= (K);
            tFlux(y,x) *= (K);
            bFlux(y,x) *= (K);
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

    // Update water surface and velocity field
    ////////////////////////////////////////////////////////////
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(uVel.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (int y=0; y<uVel.height(); ++y)
#endif
    {
        for (int x=0; x<uVel.width(); ++x)
        {
            float inFlow = getRFlux(y,x-1) + getLFlux(y,x+1) + getTFlux(y-1,x) + getBFlux(y+1,x);
            float outFlow = getRFlux(y,x) + getLFlux(y,x) + getTFlux(y,x) + getBFlux(y,x);
            float dV = dt*(inFlow-outFlow);
            float oldWater = water(y,x);
            float actualWater = water(y,x);
            //water(y,x) += dV/(dx*dy);
            //water(y,x) = std::max(water(y,x),0.0f);
            actualWater += dV/(dx*dy);
            actualWater = std::max(actualWater,0.0f);
            if(actualWater>0)
                counter_from_last_time_water_passed(y,x)=0;
            else
                counter_from_last_time_water_passed(y,x)++;

            float meanWater = 0.5*(oldWater+ actualWater);

            if (meanWater == 0.0f)
            {
                uVel(y,x) = vVel(y,x) = 0.0f;
            }
            else
            {
                uVel(y,x) =  0.5*(getRFlux(y,x-1)-getLFlux(y,x)-getLFlux(y,x+1)+getRFlux(y,x))/(dy*meanWater);
                vVel(y,x) =  0.5*(getTFlux(y-1,x)-getBFlux(y,x)-getBFlux(y+1,x)+getTFlux(y,x))/(dx*meanWater);
            }

            //we define lakes and seas as places where the incoming and the outcoming water flux is almost zero
           /* if( inFlow  >= 0 - still_water_treshold && inFlow  <= 0 + still_water_treshold &&
                outFlow  >= 0 - still_water_treshold && outFlow  <= 0 + still_water_treshold && water(y,x)>ocean_level)
            {
                counter_times_water_was_still(y,x)++;
            }
            else
            {
                if(counter_times_water_was_still(y,x) < treshold_time )
                {
                    counter_times_water_was_still(y,x) = 0;
                }
            }
            */
water(y,x) = actualWater;/*
            float uV = uVel(y,x);
            float vV = vVel(y,x);
            count(y,x)++;

            float vel = sqrtf(uV*uV+vV*vV);
            

           if(vel <= river_max_speed_treshold && vel>= river_min_speed_treshold && water(y,x)>river_min_height_treshold &&
                    water(y,x) <= river_height_treshold)
            {

              //  counter_times_water_was_still(y,x)++;
               float temp_count = counter_times_water_was_still(y,x);
               //std::cout<<temp_count<<std::endl;
//                counter_times_water_was_still(y,x) = counter_times_water_was_still(y,x)>treshold_time? treshold_time +1:counter_times_water_was_still(y,x)++;
                if(temp_count>treshold_time){
                  //  state.simData_2(y,x) =1;
                    counter_times_water_was_still(y,x) = treshold_time+1;
                }
                else{
                    counter_times_water_was_still(y,x)++;
                }

            }
            else
            {
                if(counter_times_water_was_still(y,x) >  treshold_time )
                {
                     state.simData_2(y,x) =1;
                    counter_times_water_was_still(y,x)++;
                    if(counter_times_water_was_still(y,x)>3000)
                        counter_times_water_was_still(y,x)=0;
                }
                else{
                    state.simData_2(y,x) =0;
                    counter_times_water_was_still(y,x)=0;

                }
            }*/
        }
    }

#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

}

// flux acess
float FluidSimulation::getRFlux(int y, int x) {
    if (x<0 || x>rFlux.width()-1) {
        return 0.0f;
    } else {
        return rFlux(y,x);
    }
}

float FluidSimulation::getLFlux(int y, int x) {
    if (x<0 || x>lFlux.width()-1) {
        return 0.0f;
    } else {
        return lFlux(y,x);
    }
}

float FluidSimulation::getBFlux(int y, int x) {
    if (y<0 || y>bFlux.height()-1) {
        return 0.0f;
    } else {
        return bFlux(y,x);
    }
}

float FluidSimulation::getTFlux(int y, int x) {
    if (y<0 || y>tFlux.height()-1) {
        return 0.0f;
    } else {
        return tFlux(y,x);
    }
}

float FluidSimulation::getRFlux_air(int y, int x) {
    if (x<0 || x>rFlux_air.width()-1) {
        return 0.0f;
    } else {
        return rFlux_air(y,x);
    }
}

float FluidSimulation::getLFlux_air(int y, int x) {
    if (x<0 || x>lFlux_air.width()-1) {
        return 0.0f;
    } else {
        return lFlux_air(y,x);
    }
}

float FluidSimulation::getBFlux_air(int y, int x) {
    if (y<0 || y>bFlux_air.height()-1) {
        return 0.0f;
    } else {
        return bFlux_air(y,x);
    }
}

float FluidSimulation::getTFlux_air(int y, int x) {
    if (y<0 || y>tFlux_air.height()-1) {
        return 0.0f;
    } else {
        return tFlux_air(y,x);
    }
}



float FluidSimulation::getTerrain(int y, int x) {
    return terrain(glm::clamp(y,0,(int) terrain.height()-1),glm::clamp(x,0,(int) terrain.width()-1));
}

float FluidSimulation::getSedimentMaterial(int y, int x) {
    return tmp_sediment_material(glm::clamp(y,0,(int) tmp_sediment_material.height()-1),glm::clamp(x,0,(int) tmp_sediment_material.width()-1));
}

float FluidSimulation::getWater(int y, int x){
    return water(glm::clamp(y,0,(int) water.height()-1),glm::clamp(x,0,(int) water.width()-1));
}

float FluidSimulation::getAir(int y, int x){
    return air(glm::clamp(y,0,(int) air.height()-1),glm::clamp(x,0,(int) air.width()-1));
}

void FluidSimulation::simulateErosion(double dt,ulong time)
{
    float Ks = 0.0001f*12*10; // dissolving constant
    float Kd = 0.0001f*12*10; // deposition constant
   // if(time>150)
   //     Kd *= 0.5;
    float max = 10;
    float min = -10;
    float scale = 3;
    float range = max-min;
    PerlinNoise perlin;
    glm::vec4 black_col(0,0,0,1);
    glm::vec4 sed_color(1,1,0,0);
        /*if(time%100>75){
        std::cout<<"new mat 6"<<std::endl;
        sediment_material = 6;
        sed_color = glm::vec4(0,1,1,1);
    }
    if(time%100>87){
        std::cout<<"new mat 7"<<std::endl;
        sediment_material = 7;
        sed_color = glm::vec4(0,1,1,1);
    }*/
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(sediment.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (int y=0; y<sediment.height(); ++y)
#endif
    {
        for (int x=0; x<sediment.width(); ++x)
        {
            // local velocity
            float uV = uVel(y,x);
            float vV = vVel(y,x);
            // local terrain normal
            vec3 normal = vec3(getTerrain(y,x+1) - getTerrain(y,x-1), getTerrain(y+1,x) - getTerrain(y-1,x), 2 );
            normal = normalize(normal);
            vec3 up(0,0,1);
            float cosa = dot(normal,up);
            float sinAlpha = sin(acos(cosa));
            sinAlpha = std::max(sinAlpha,0.1f);

            // local sediment capacity of the flow
            //better keep this number not too high
            //std::cout<<"range"<<range<<std::endl;
            //std::cout<<"num strat "<< terrain_data.number_of_stratifications<<std::endl;
            //int z =Kc *             float stratification_level = 1;//=  interval * roundf(z/interval);
            //int index = stratification_level/interval;
            //float Kc =  terrain_data.list_of_rock_capacity_values.at(index);
            // sediment capacity constant
            float z = getTerrain(y,x);
            float kc = 0;
            float n = (perlin.Sample(noise_sediment_frequency*x,noise_sediment_frequency*y,noise_sediment_frequency*z));

            //int res = _stratified_layer_width*(roundf(z/_stratified_layer_width));
            float level = sin((z+z/30*n)*_stratified_layer_width);
             if( level> 0)
                 kc = 35;
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
            kc *= 2;
            state.simData(y,x) = kc/42;
            //if(z>15)
            //    kc =15;
            //else
            int sed_mat = roundf(tmp_sediment_material(y,x));
            switch (sed_mat) {
            case 1 :
            {
                kc = 18;
                break;
            }
            case 2:
            {
                kc = 20;
                break;
            }
            case 3:
            {
                kc = 28;
                break;
            }
            case 4:
            {
                kc = 36;
                break;
            }
            case 5:
            {
                kc = 44;
                break;
            }

            default:
                kc = 50;
            }
            float capacity = kc* sqrtf(uV*uV+vV*vV)*sinAlpha*(std::min(water(y,x),0.01f)/0.01f) ;
            float delta = (capacity-sediment(y,x));

            //float v = sqrtf(uV*uV+vV*vV);
            //float fctr = (std::min(water(y,x),0.01f)/0.01f);
            sedimented_terrain(y,x) = 0;

            if (delta > 0.0f)
            {
                float d = Ks*delta;
                terrain(y,x)  -= d;
                water(y,x)    += d;
                sediment(y,x) += d;
              //  if(water(y,x)>0.01)
              //  tmp_sediment_material(y,x) = sedimented_material(y,x);
                    sedimented_terrain(y,x) -= d;
                  //if(tmp_sediment_material(y,x)!=0)
                  //  tmp_sediment_material(y,x) = 0;
//                    sedimented_material(y,x) = 0;
                //tmp_sediment_material(y,x) = sediment_material;
                //sedimented_terrain(y,x) -= sediment_material;
              // if( sedimented_terrain_color(x,y) == black_col)
        //           sedimented_terrain_color(x,y)  = sed_color;
            }
            // deposit onto ground
            else if (delta < 0.0f /*&& water(y,x)!=0 */)
            {
                float d = Kd*delta;
                terrain(y,x)  -= d;
                water(y,x)    += d;
                sediment(y,x) += d;
               sedimented_terrain(y,x) -= d;//
              // sedimented_material(y,x) = tmp_sediment_material(y,x);
          //     tmp_sediment_material(y,x) = 0;
               //    if( sedimented_terrain_color(y,x) == black_col)

                    sedimented_terrain_color(y,x)  = sed_color;
            }
        //    if(sediment(y,x)<0){
        //        sedimented_material(y,x)=0;
        //        tmp_sediment_material(y,x) = 0;
        //    }
            //else{
            //    tmp_sediment_material(y,x) = 0;
            //}
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif
}

float FluidSimulation::GetTerrainCapacity(float x,float y,float z ,float frequency,int* layers )
{
    PerlinNoise perlin;

    int level = 0;
    float kc = 0;
    if(z<30)
    {
       if(z<18)
       {
           if(z<12)
           {
               if(z<6)
               {
                  level=1;
               }
               else
               {
                   level = 2;
               }
           }
           else
           {
               level = 3;
           }
       }
       else
       {
           level = 4;
       }

    }
    else
        level = 5;

    float n = (0.5+0.5*perlin.Sample(frequency*x,frequency*y,frequency*z));
  //  n = 0.51;
    if(n<0.25)
    {
        kc = layers[level-1];
    }
    else
    {
        if(n<0.5)
        {
            kc = layers[level+1];
        }else
        {
            kc = layers[level];
        }
    }
return kc;
}

bool sortByVal(const pair<int, float> &a,
               const pair<int, float> &b)
{
    return (a.second < b.second);
}


void FluidSimulation::simulateSedimentTransportation(double dt,ulong time)
{
    std::uniform_int_distribution<int> rndFloat(0,INT_MAX);
    // semi-lagrangian advection
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(sediment.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<sediment.height(); ++y)
#endif
    {
        for (uint x=0; x<sediment.width(); ++x)
        {
            // local velocity
            float uV = uVel(y,x);
            float vV = vVel(y,x);

            // position where flow comes from
            float fromPosX = float(x) - uV*dt;
            float fromPosY = float(y) - vV*dt;

            // integer coordinates
            int x0 = Floor2Int(fromPosX);
            int y0 = Floor2Int(fromPosY);
            int x1 = x0+1;
            int y1 = y0+1;

            // interpolation factors
            float fX = fromPosX - x0;
            float fY = fromPosY - y0;

            // clamp to grid borders
            x0 = clamp(x0,0,int(sediment.width()-1));
            x1 = clamp(x1,0,int(sediment.width()-1));
            y0 = clamp(y0,0,int(sediment.height()-1));
            y1 = clamp(y1,0,int(sediment.height()-1));

            float newVal = mix( mix(sediment(y0,x0),sediment(y0,x1),fX), mix(sediment(y1,x0),sediment(y1,x1),fX), fY);
            int material_1 = (int)tmp_sediment_material(y0,x0);
            int material_2 = (int)tmp_sediment_material(y0,x1);
            int material_3 = (int)tmp_sediment_material(y1,x0);
            int material_4 = (int)tmp_sediment_material(y1,x1);
            std::map<int,float> map_of_materials;
            int target_id = 3;
            bool mat_4 = false;
            float sediment_quantity;

          //  sediment_quantity = x0*x0+y0*y0;
            sediment_quantity = fX*fX+fY*fY;

          //  if(sedimented_terrain(y0,x0)<0)
          //      sediment_quantity *= 1;
          //  else
          //      sediment_quantity *= 0;
            map_of_materials.insert(std::make_pair(material_1,sediment_quantity));
           // sediment_quantity = (1-x0)*(1-x0)+y0*y0;
             sediment_quantity = (1-fX)*(1-fX)+fY*fY;
            //if(sedimented_terrain(y0,x1)<0)
            //    sediment_quantity *= 1;
            //else
            //    sediment_quantity *= 0;

            if(map_of_materials.count(material_2)>0){
                map_of_materials[material_2]+=sediment_quantity;
            }else{
                map_of_materials.insert(make_pair(material_2,sediment_quantity));
            }

       //     sediment_quantity = x0*x0+(1-y0)*(1-y0);
            sediment_quantity = fX*fX+(1-fY)*(1-fY);
           // if(sedimented_terrain(y1,x0)<0)
           //     sediment_quantity *= 1;
           // else
           //     sediment_quantity *= 0;
            if(map_of_materials.count(material_3)>0){
                map_of_materials[material_3]+=sediment_quantity;
            }else{
                map_of_materials.insert(make_pair(material_3,sediment_quantity));
            }
//            sediment_quantity = (1-x0)*(1-x0)+(1-y0)*(1-y0);
            sediment_quantity = (1-fX)*(1-fX)+(1-fY)*(1-fY);
         //  if(sedimented_terrain(y1,x1)<0)
         //      sediment_quantity *= 1;
         //  else
         //      sediment_quantity *= 0;
            if(map_of_materials.count(material_4)>0){
                map_of_materials[material_4]+=sediment_quantity;
            }else{
                map_of_materials.insert(make_pair(material_4,sediment_quantity));
            }
            float newValMat = 0;
            float max = -INFINITY;
            float total = 0;
            bool roi =false;

          //  if(x<sediment.height()*2/3+5 && x>sediment.height()*2/3-5){
          //      roi = true;
          //  }

            for(std::pair<int,float> entry: map_of_materials){
                total+= entry.second;
                if(mat_4 && roi){
                    std::cout<<" id "<<entry.first<<" quantity"<<entry.second<<std::endl;
                }

               // if(entry.second<0 || entry.second>1)
               //     std::cout<<"entry "<<entry.second<<std::endl;
                if(entry.second>max ){
                    max = entry.second;
                    newValMat = entry.first;
                }
            }
            if(mat_4 && roi){
                std::cout<<" end "<<std::endl;
            }

       //total = -1;
            if(total>0)    {
            float new_total = 0;
            std::vector<std::pair<int,float>> list_of_probabilities;
             map<int, float> :: iterator it_map;
            for (it_map=map_of_materials.begin(); it_map!=map_of_materials.end(); it_map++)
            {

                float new_val = it_map->second;
                new_val = new_val/(float)total;
                new_total += new_val;
              list_of_probabilities.push_back(make_pair(it_map->first,new_val ));
            }
            std::sort(list_of_probabilities.begin(), list_of_probabilities.end(), sortByVal);
            total = 0;
            for(auto& entry:list_of_probabilities){
              entry.second += total;
              total = entry.second;
            }
    int newValMat2= 0;
    new_total = 0;
    total = 0;
            float sample = (float) rndFloat(rnd)/(float)INT_MAX;
            glm::clamp(sample,0.0f,1.0f);
            for(auto const entry:list_of_probabilities){
                if(sample< entry.second){
               newValMat2 = entry.first;
                    break;
                }
           }
           if(newValMat2==0){
               newValMat2 =  list_of_probabilities.back().second;
           }
           newValMat= newValMat2;
           }
            if(uV == 0 || vV==0)
                newValMat = -1;

            //else
         /*    //   std::cout<<"total is "<<total<<std::endl;
            float val = 0;
            if(material_1>0){
                val = material_1;
                material_1 = 1;
            }
            if(material_2>0){
                val = material_2;
                material_2 = 1;
            }
            if(material_3>0){
                val = material_3;
                material_3 = 1;
            }
            if(material_4>0){
                val = material_4;
                material_4 = 1;
            }*/

           newValMat = mix( mix((float)material_1,(float)material_2,fX), mix((float)material_3,(float)material_4,fX), fY);

         //  newValMat = mix( mix(tmp_sediment_material(y0,x0),tmp_sediment_material(y0,x1),fX), mix(tmp_sediment_material(y1,x0),tmp_sediment_material(y1,x1),fX), fY);
    //newValMat = roundf(newValMat)*val;
        //   if(time>250)
        //        newValMat = 0;
            tmpSediment(y,x) = newVal;
            if(newValMat!=-1)
                tmp_sediment_material_2(y,x) = roundf(newValMat);
            else
                tmp_sediment_material_2(y,x) = tmp_sediment_material(y,x);

    }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif

    // write back new values
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(sediment.size(), gcdq, ^(size_t i)
#else
    #pragma omp parallel for
    for (uint i=0; i<sediment.size(); ++i)
#endif
    {
    // if(tmpSediment(i)- sediment(i)>0.001)
        tmp_sediment_material(i) = tmp_sediment_material_2(i);
        sediment(i) = tmpSediment(i);
        /*if(tmpSediment(i)==0){
            sedimented_material(i) == 0;
        }*/
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif
}

void FluidSimulation::simulateEvaporation(double dt)
{
    const float Ke = 0.00011*0.5;//0.00011*0.5; // evaporation constant
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(water.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<water.height(); ++y)
#endif
    {
        for (uint x=0; x<water.width(); ++x)
        {
            water(y,x) = std::max(water(y,x)*(1-Ke*dt),0.0);

            if (water(y,x) <0.005f)
            {
                water(y,x) = 0.0f;
            }
        }
    }
#if defined(__APPLE__) || defined(__MACH__)
    );
#endif
}

void FluidSimulation::EraseAll(){
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(water.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<water.height(); ++y)
#endif
    {
        for (uint x=0; x<water.width(); ++x)
        {
                water(y,x) = 0;
                tmp_sediment_material(y,x)=0;
                sediment(y,x) = 0;
                sedimented_material(y,x) = 0;
        }
        }
}


void FluidSimulation::EraseWater(){
#if defined(__APPLE__) || defined(__MACH__)
    dispatch_apply(water.height(), gcdq, ^(size_t y)
#else
    #pragma omp parallel for
    for (uint y=0; y<water.height(); ++y)
#endif
    {
        for (uint x=0; x<water.width(); ++x)
        {
            if(x > water.width()-5)
                water(y,x) = 0;
        }
        }
}

void FluidSimulation::update(ulong time, double dt, bool rain, bool flood,bool wind)
{
  // if(time%40>30 || time>50*4-1)
  //     rain= false;
  // else
  //     rain = true;

    // 1. Add water to the system
    //if ( rain && time<250 )
    //time%10<5 && time<250
    ulong time_x = 100;
//    if(rain)
//        makeRain(dt,time);
    if( time<time_x+1){
        if(time%20<15)
            makeRain(dt,time);
        if(time%20==0)
            EraseAll();
    }

    if(time == time_x+1){
        EraseAll();
    }
    if(time>time_x+2)
        makeRiver(dt,time);
        //makeFlood(dt,time);
//    if (flood || time>260)
//       makeRiver(dt,time);// makeFlood(dt);
    if (flood )
           makeRiver(dt,time);// makeFlood(dt);
    if(wind)
        makeWind(dt);
    // 2. Simulate Flow
    simulateFlow(dt);

    simulateWind(dt);
   // simulateWind(dt);

    // 3. Simulate Errosion-deposition
    simulateErosion(dt,time);
    // 4. Advection of suspended sediment
    simulateSedimentTransportation(dt,time);
    // 5. Simulate Evaporation
    simulateEvaporation(dt);

    smoothTerrain();
    computeSurfaceNormals();
    EraseWater();

}
