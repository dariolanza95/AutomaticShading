/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef STATE_H
#define STATE_H

#include "Grid2D.h"
#include "Grid3D.h"
#include "platform_includes.h"

#include "Math/PerlinNoise.h"

using namespace glm;

class SimulationState
{
public:
    Grid2D<float> water;
    Grid2D<float> air;
    Grid2D<ulong> air_total_pressure;
    Grid2D<ulong> air_counter;
    Grid2D<float> terrain;
    Grid2D<float> suspendedSediment;
    Grid2D<float> vegetation;
    Grid2D<float> simData;
    Grid2D<float> simData_2;
    Grid2D<float> rivers;
    Grid2D<float> sedimented_terrain;
    Grid2D<float> sedimented_material;
    Grid2D<float> temp_sedimented_material;
    Grid2D<glm::vec4> sedimented_terrain_color;
    //Grid2D<vec3>  flowNormal;
    glm::vec2 windDirection;
    Grid2D<vec3> surfaceNormals;
public:

    SimulationState(uint w, uint h,glm::vec2 WindDirection = glm::vec2(0,1))
        :   water(w,h),
            air(w,h),
            air_total_pressure(w,h),
            air_counter(w,h),
            terrain(w,h),
            suspendedSediment(w,h),
            sedimented_terrain(w,h),
            sedimented_material(w,h),
            sedimented_terrain_color(w,h),
            surfaceNormals(w,h),
            vegetation(w,h),
            rivers(w,h),
            simData(w,h),simData_2(w,h),
            temp_sedimented_material(w,h),
            windDirection(WindDirection)
            //flowNormal(w,h)
    {

        //createPerlinTerrain();
       createRiverTerrain();
       //createSteepTerrain();
       //createSeaTerrain();
        //createSedimentationTerrain();
        int l = 20;
        int mw = 20;
        int xmin = water.width() - l-mw;
        int xmax = water.width() - l;

        int ymin = water.height() - l-mw;
        int ymax = water.height() - l;

    }

void createSeaTerrain(){
    glm::normalize(windDirection);
    PerlinNoise perlin;
    float angle1 = 0;
    float offset = 20;
    float angle2 = 0;
    for (uint y=0; y<water.height(); y++)
    {
        for (uint x=0; x<water.width(); x++)
        {
            water(y,x) = 0.0f;
            air(y,x) = 0.0f;
            air_counter(y,x)=0;
            air_total_pressure(y,x) = 0;
            suspendedSediment(y,x) = 0.0f;
            vegetation(y,x) = 0.0f;
            simData(y,x) = 0.0f;
             simData_2(y,x) = 0.0f;
            rivers(y,x) = 0.0f;
            if (x < (water.height()*1/10))
            {
                if (y > (water.height()/2))
                {
                    terrain(y,x) =offset+ (y-water.height()/2) * tan(M_PI*angle1/180);
                                        }
                else
                {
                    terrain(y,x) = offset+ (-y+water.height()/2) * tan(M_PI*angle1/180);
                    //terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                }
                float temp= (-x+water.height()*1/10) * tan(M_PI*angle2/180);
               //temp = 0;
                terrain (y,x) = std::max(terrain(y,x),temp);
                //terrain (y,x) = std::min(terrain(y,x),50);

                sedimented_terrain(y,x) = 0;
                sedimented_material(y,x)= 0;
                sedimented_terrain_color(y,x) = glm::vec4(0,0,0,1);
                temp_sedimented_material(y,x) = 0;

            }
            else
            {
               // terrain(y,x) = -10;
                 terrain(y,x) = 0;
            }

        }
    }
}


    void createRiverTerrain()
    {
        glm::normalize(windDirection);
        PerlinNoise perlin;
        float angle1 = 20;
        float offset = 20;
        float angle2 = 5;
        for (uint y=0; y<water.height(); y++)
        {
            for (uint x=0; x<water.width(); x++)
            {
                water(y,x) = 0.0f;
                air(y,x) = 0.0f;
                air_counter(y,x)=0;
                air_total_pressure(y,x) = 0;
                suspendedSediment(y,x) = 0.0f;
                vegetation(y,x) = 0.0f;
                simData(y,x) = 0.0f;
                 simData_2(y,x) = 0.0f;
                rivers(y,x) = 0.0f;
                if (x < (water.height()*2/3))
                {
                    if (y > (water.height()/2))
                    {
                        terrain(y,x) =offset+ (y-water.height()/2) * tan(M_PI*angle1/180);
                                            }
                    else
                    {
                        terrain(y,x) = offset+ (-y+water.height()/2) * tan(M_PI*angle1/180);
                        //terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                    }
                    float temp= (-x+water.height()*2/3) * tan(M_PI*angle2/180);
                    terrain (y,x) = std::max(terrain(y,x),temp);
                    sedimented_terrain(y,x) = 0;
                    sedimented_material(y,x)= 0;
                    sedimented_terrain_color(y,x) = glm::vec4(0,0,0,1);
                    temp_sedimented_material(y,x) = 0;

                }
                else
                {
                    terrain(y,x) = -10;
                }

            }
        }
    }

    void createSedimentationTerrain()
    {
        glm::normalize(windDirection);
        PerlinNoise perlin;
        float angle1 = 30;
        float offset = 40;
        float angle2 = 30;
        for (uint y=0; y<water.height(); y++)
        {
            for (uint x=0; x<water.width(); x++)
            {
                sedimented_terrain(y,x) = 0;
                sedimented_material(y,x)= 0;
                sedimented_terrain_color(y,x) = glm::vec4(0,0,0,1);
                temp_sedimented_material(y,x) = 0;
                water(y,x) = 0.0f;
                air(y,x) = 0.0f;
                air_counter(y,x)=0;
                air_total_pressure(y,x) = 0;
                suspendedSediment(y,x) = 0.0f;
                vegetation(y,x) = 0.0f;
                simData(y,x) = 0.0f;
                 simData_2(y,x) = 0.0f;
                rivers(y,x) = 0.0f;
                float h_temp=0;
                float temp = 0;
                float final_h = 0;
                if (x < (water.height()*1/3))
                {
                    if (y > (water.height()/2))
                    {
                        terrain(y,x) =offset+ (y-water.height()/2) * tan(M_PI*angle1/180);
                                            }
                    else
                    {
                        terrain(y,x) = offset+ (-y+water.height()/2) * tan(M_PI*angle1/180);
                        //terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                    }
                    float temp= (-x+water.height()*1/3) * tan(M_PI*angle2/180);
                    terrain (y,x) = std::max(terrain(y,x),temp);
                    sedimented_terrain(y,x) = 0;
                    sedimented_material(y,x)= 0;
                    sedimented_terrain_color(y,x) = glm::vec4(0,0,0,1);
                    temp_sedimented_material(y,x) = 0;

                }

               //temp = 0;
                //terrain (y,x) = temp;

                if (x < (water.height()*1/3))
                {
                  /*  if (y > (water.height()/2))
                    {
                        terrain(y,x) =offset+ (y-water.height()/2) * tan(M_PI*angle1/180);
                                            }
                    else
                    {
                        terrain(y,x) = offset+ (-y+water.height()/2) * tan(M_PI*angle1/180);
                        //terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                    }
                    float temp= (-x+water.height()*2/3) * tan(M_PI*angle2/180);
                   //temp = 0;
                    //terrain (y,x) = temp;
                    terrain (y,x) = std::max(terrain(y,x),temp);*/
                    //terrain (y,x) = std::max(terrain(y,x),temp);
                    //terrain (y,x) = std::min(terrain(y,x),50);
                    //terrain(y,x) = final_h;


                }
                else
                {
                    //if(x < (water.height()*2/3)){
                    //    glm::vec2 a ();
                    //    glm::vec2 b (water.height()*2/3,y);
                    //    float temp= (-x+water.height()*2/3) * tan(M_PI*angle2/180);
                    //   //temp = 0;
                    //    terrain (y,x) = std::min(temp,final_h);
                    //}else
                    {

                        terrain(y,x) = 0;
                    }
                }

            }
        }
    }


    void createPerlinTerrain()
    {
        PerlinNoise perlin;
        float max = 0;
        for (uint y=0; y<water.height(); y++)
        {
            for (uint x=0; x<water.width(); x++)
            {
                water(y,x) = 0.0f;
                air(y,x) = 0.0f;
                air_counter(y,x)=0;
                air_total_pressure(y,x) = 0;
                //flowNormal(y,x) = 0.0f;
                float h = 0.0f; float f = 0.05f;
                h += perlin.Sample(y*f,x*f)*1; f /= 2;
                h += perlin.Sample(y*f,x*f)*2; f /= 2;
                h += perlin.Sample(y*f,x*f)*4; f /= 2;
                h += perlin.Sample(y*f,x*f)*8; f /= 2;
                max = h>max ? h:max;
                terrain(y,x) = h*4*1.3;
                suspendedSediment(y,x) = 0.0f;// 0.1*terrain(y,x);
                vegetation(y,x) = 0.0f;
                simData(y,x) = 0.0f;
                simData_2(y,x) = 0.0f;
                rivers(y,x) = 0.0f;

            }
        }
    }



    void createSteepTerrain()
    {
        for (uint y=0; y<water.height(); y++)
        {
            for (uint x=0; x<water.width(); x++)
            {
                water(y,x) = 0.0f;
                air(y,x) = 0.0f;
                if (x > (water.height()/2)) {
                    terrain(y,x) = 0.2*(x-water.height()/2);
                } else {
                    terrain(y,x) = 0.2*(-x+water.height()/2);
                }
                if (y > (water.height()/2)) {
                    terrain(y,x) = std::max(terrain(y,x),0.2f*(y-water.height()/2));
                } else {
                    terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                }
                sedimented_terrain(y,x) = 0;
                sedimented_material(y,x)= 0;
                sedimented_terrain_color(y,x) = glm::vec4(0,0,0,1);
                temp_sedimented_material(y,x) = 0;

                suspendedSediment(y,x) = 0.0f;
            }
        }
    }


};

#endif // STATE_H
