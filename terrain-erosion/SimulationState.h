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
    Grid2D<float> terrain;
    Grid2D<float> suspendedSediment;
    Grid2D<float> vegetation;
    Grid2D<float> simData;
    Grid2D<float> simData_2;
    Grid2D<float> rivers;

    //Grid2D<vec3>  flowNormal;

    Grid2D<vec3> surfaceNormals;
public:

    SimulationState(uint w, uint h)
        :   water(w,h),
            terrain(w,h),
            suspendedSediment(w,h),
            surfaceNormals(w,h),
            vegetation(w,h),
            rivers(w,h),
            simData(w,h),simData_2(w,h)
            //flowNormal(w,h)
    {

        //createPerlinTerrain();
        createRiverTerrain();
        int l = 20;
        int mw = 20;
        int xmin = water.width() - l-mw;
        int xmax = water.width() - l;

        int ymin = water.height() - l-mw;
        int ymax = water.height() - l;

    }




    void createRiverTerrain()
    {
        PerlinNoise perlin;
        float angle1 = 20;
        float angle2 = 10;
        for (uint y=0; y<water.height(); y++)
        {
            for (uint x=0; x<water.width(); x++)
            {
                water(y,x) = 0.0f;
                suspendedSediment(y,x) = 0.0f;
                vegetation(y,x) = 0.0f;
                simData(y,x) = 0.0f;
                 simData_2(y,x) = 0.0f;
                rivers(y,x) = 0.0f;
                if (x < (water.height()*2/3))
                {
                    if (y > (water.height()/2))
                    {
                        terrain(y,x) = (y-water.height()/2) * tan(M_PI*angle1/180);
                                            }
                    else
                    {
                        terrain(y,x) = (-y+water.height()/2) * tan(M_PI*angle1/180);
                        //terrain(y,x) = std::max(terrain(y,x),0.2f*(-y+water.height()/2));
                    }
                    float temp= (-x+water.height()*2/3) * tan(M_PI*angle2/180);
                    terrain (y,x) = std::max(terrain(y,x),temp);

                }
                else
                {
                    terrain(y,x) = -10;
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

                suspendedSediment(y,x) = 0.0f;
            }
        }
    }


};

#endif // STATE_H
