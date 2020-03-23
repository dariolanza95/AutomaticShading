/*
# ------------------------------------------------------------------------------
#
# Copyright (c) 1986-2019 Pixar. All rights reserved.
#
# The information in this file (the "Software") is provided for the exclusive
# use of the software licensees of Pixar ("Licensees").  Licensees have the
# right to incorporate the Software into other products for use by other
# authorized software licensees of Pixar, without fee. Except as expressly
# permitted herein, the Software may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior written
# permission of Pixar.
#
# The copyright notices in the Software and this entire statement, including the
# above license grant, this restriction and the following disclaimer, must be
# included in all copies of the Software, in whole or in part, and all permitted
# derivative works of the Software, unless such copies or derivative works are
# solely in the form of machine-executable object code generated by a source
# language processor.
#
# PIXAR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL PIXAR BE
# LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
# OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
# CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.  IN NO CASE WILL
# PIXAR'S TOTAL LIABILITY FOR ALL DAMAGES ARISING OUT OF OR IN CONNECTION WITH
# THE USE OR PERFORMANCE OF THIS SOFTWARE EXCEED $50.
#
# Pixar
# 1200 Park Ave
# Emeryville CA 94608
#
# ------------------------------------------------------------------------------
*/

#include "RixPredefinedStrings.hpp"
#include "RixPattern.h"
#include "RixShadingUtils.h"
#include "pointcloud.h"
#include <math.h>
#include <bits/stdc++.h>


class PxrWorleyD : public RixPattern
{
public:

    PxrWorleyD();
    virtual ~PxrWorleyD();

    virtual int Init(RixContext &, RtUString const pluginpath) override;
    virtual RixSCParamInfo const *GetParamTable() override;

    virtual void Synchronize(
        RixContext&,
        RixSCSyncMsg,
        RixParameterList const*) override
    {
    }

    virtual void Finalize(RixContext &) override;
    float Fbm(RtPoint3 cell,int octaves,float amplitude );
    virtual int ComputeOutputParams(RixShadingContext const *,
                                    RtInt *noutputs,
                                    OutputSpec **outputs,
                                    RtPointer instanceData,
                                    RixSCParamInfo const *) override;

    virtual bool Bake2dOutput(
        RixBakeContext const*,
        Bake2dSpec&,
        RtPointer) override
    {
        return false;
    }

    virtual bool Bake3dOutput(
        RixBakeContext const*,
        Bake3dSpec&,
        RtPointer) override
    {
        return false;
    }

private:
    // Defaults
    RtInt const m_surfacePosition;
    RtFloat const m_frequency;
    RtInt const m_distancemetric;
    RtFloat const m_jitter;
    RtInt const m_clamp;
    RtFloat const m_c1;
    RtFloat const m_c2;
    RtFloat const m_minkowskiExp;
    RtInt const m_shape;
    RtFloat const m_randomScale;
    RtFloat const m_randomScaleCenter;
    RtInt const m_invert;
    RtColorRGB const m_colorScale;
    RtColorRGB const m_colorOffset;
    RtFloat const m_floatScale;
    RtFloat const m_floatOffset;
    RtPoint3 const m_defaultST;

    RixShadeFunctions *m_sFuncs;
};

PxrWorleyD::PxrWorleyD() :
    m_surfacePosition(0.2),
    m_frequency(0.20f),
    m_distancemetric(0.4),
    m_jitter(0.75f),
    m_clamp(1),
    m_c1(0.8f),
    m_c2(-0.2f),
    m_minkowskiExp(4.0f),
    m_shape(0),
    m_randomScale(1.0f),
    m_randomScaleCenter(0.0f),
    m_invert(0),
    m_colorScale(1.f, 1.f, 1.f),
    m_colorOffset(0.f, 0.f, 0.f),
    m_floatScale(1.f),
    m_floatOffset(0.f),
    m_defaultST(0.0f, 0.0f, 0.0f),
    m_sFuncs(NULL)
{
}

PxrWorleyD::~PxrWorleyD()
{
}

int
PxrWorleyD::Init(RixContext &ctx, RtUString const pluginpath)
{
    PIXAR_ARGUSED(pluginpath);

    m_sFuncs = (RixShadeFunctions*)ctx.GetRixInterface(k_RixShadeFunctions);
    if (!m_sFuncs)
        return 1;
    else
        return 0;
}

enum paramId
{
    k_resultF = 0,
    k_resultDispl,
    k_resultRGB,
    k_surfacePosition,
    k_frequency,
    k_distancemetric,
    k_jitter,
    k_clamp,
    k_c1,
    k_c2,
    k_minkowskiExponent,
    k_shape,
    k_randomScale,
    k_randomScaleCenter,
    k_invert,
    k_colorScale,
    k_colorOffset,
    k_floatScale,
    k_floatOffset,
    k_manifold,
    k_manifoldQ,
    k_manifoldQradius,
    k_manifoldEnd
};

enum distanceMetric
{
    k_euclidean,
    k_euclideanSquared,
    k_manhattan,
    k_chebyshev,
    k_minkowski
};

enum cellShape
{
    k_linear = 0,
    k_thin,
    k_fat
};

enum surfPos
{
    k_useP = 0,
    k_usePo
};

RixSCParamInfo const *
PxrWorleyD::GetParamTable()
{
    static RixSCParamInfo s_ptable[] =
    {
        RixSCParamInfo(RtUString("resultF"), k_RixSCFloat, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultDispl"), k_RixSCFloat, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultRGB"), k_RixSCColor, k_RixSCOutput),
        RixSCParamInfo(RtUString("surfacePosition"), k_RixSCInteger),
        RixSCParamInfo(RtUString("frequency"), k_RixSCFloat),
        RixSCParamInfo(RtUString("distancemetric"), k_RixSCInteger),
        RixSCParamInfo(RtUString("jitter"), k_RixSCFloat),
        RixSCParamInfo(RtUString("clamp"), k_RixSCInteger),
        RixSCParamInfo(RtUString("c1"), k_RixSCFloat),
        RixSCParamInfo(RtUString("c2"), k_RixSCFloat),
        RixSCParamInfo(RtUString("minkowskiExponent"), k_RixSCFloat),
        RixSCParamInfo(RtUString("shape"), k_RixSCInteger),
        RixSCParamInfo(RtUString("randomScale"), k_RixSCFloat),
        RixSCParamInfo(RtUString("randomScaleCenter"), k_RixSCFloat),
        RixSCParamInfo(RtUString("invert"), k_RixSCInteger),

        RixSCParamInfo(RtUString("colorScale"), k_RixSCColor),
        RixSCParamInfo(RtUString("colorOffset"), k_RixSCColor),
        RixSCParamInfo(RtUString("floatScale"), k_RixSCFloat),
        RixSCParamInfo(RtUString("floatOffset"), k_RixSCFloat),

        RixSCParamInfo(RtUString("PxrManifold"), RtUString("manifold"), k_RixSCStructBegin),
        RixSCParamInfo(RtUString("Q"), k_RixSCPoint),
        RixSCParamInfo(RtUString("Qradius"), k_RixSCFloat),
        RixSCParamInfo(RtUString("PxrManifold"), RtUString("manifold"), k_RixSCStructEnd),
        RixSCParamInfo() // end of table
    };
    return &s_ptable[0];
}

void
PxrWorleyD::Finalize(RixContext &ctx)
{
    PIXAR_ARGUSED(ctx);
}

float PxrWorleyD::Fbm(RtPoint3 cell,int octaves,float amplitude ) {
    float v = 0.0;
    RtVector3 shift = RtVector3(100,100,100);
    for (int i = 0; i < octaves; ++i) {
        v += amplitude * m_sFuncs->Noise(cell);
        cell =  cell * 2.0 + shift;
        amplitude *= 0.5;
    }
    return v;
}



int
PxrWorleyD::ComputeOutputParams(RixShadingContext const *sctx,
                               RtInt *noutputs, OutputSpec **outputs,
                               RtPointer instanceData,
                               RixSCParamInfo const *ignored)
{
    PIXAR_ARGUSED(instanceData);
    PIXAR_ARGUSED(ignored);

    bool varying = true;
    bool uniform = false;

    RtInt const *surfacePosition;
    sctx->EvalParam(k_surfacePosition, -1, &surfacePosition,
                    &m_surfacePosition, uniform);

    RtFloat const *frequency;
    sctx->EvalParam(k_frequency, -1, &frequency, &m_frequency, varying);

    RtInt const *distancemetricp;
    sctx->EvalParam(k_distancemetric, -1, &distancemetricp,
                    &m_distancemetric, uniform);
    RtInt const distancemetric(*distancemetricp);

    RtFloat const *jitter;
    sctx->EvalParam(k_jitter, -1, &jitter, &m_jitter, varying);

    RtInt const *clampPtr;
    sctx->EvalParam(k_clamp, -1, &clampPtr, &m_clamp, uniform);
    RtInt const clamp(*clampPtr);

    RtFloat const *c1;
    sctx->EvalParam(k_c1, -1, &c1, &m_c1, varying);

    RtFloat const *c2;
    sctx->EvalParam(k_c2, -1, &c2, &m_c2, varying);

    RtFloat const *minkowskiExpPtr;
    sctx->EvalParam(k_minkowskiExponent, -1, &minkowskiExpPtr,
                    &m_minkowskiExp, uniform);
    RtFloat const minkowskiExp(RixMax(0.5f, *minkowskiExpPtr));

    RtInt const *shapePtr;
    sctx->EvalParam(k_shape, -1, &shapePtr, &m_shape, uniform);
    RtInt const shape(*shapePtr);

    RtFloat const *randomScale;
    sctx->EvalParam(k_randomScale, -1, &randomScale, &m_randomScale,
                    varying);

    RtFloat const *randomScaleCenter;
    sctx->EvalParam(k_randomScaleCenter, -1, &randomScaleCenter,
                    &m_randomScaleCenter, varying);

    RtInt const *invertPtr;
    sctx->EvalParam(k_invert, -1, &invertPtr, &m_invert, uniform);
    RtInt const invert(*invertPtr);

    RtColorRGB const *colorScale;
    sctx->EvalParam(k_colorScale, -1, &colorScale, &m_colorScale, true);
    RtColorRGB const *colorOffset;
    sctx->EvalParam(k_colorOffset, -1, &colorOffset, &m_colorOffset, true);
    RtFloat const *floatScale;
    sctx->EvalParam(k_floatScale, -1, &floatScale, &m_floatScale, true);
    RtFloat const *floatOffset;
    sctx->EvalParam(k_floatOffset, -1, &floatOffset, &m_floatOffset, true);

    // Allocate and bind our outputs
    RixShadingContext::Allocator pool(sctx);
    OutputSpec *o = pool.AllocForPattern<OutputSpec>(3);
    *outputs = o;
    *noutputs = 3;




    RtFloat *resultF = NULL;
    resultF = pool.AllocForPattern<RtFloat>(sctx->numPts);
    o[0].paramId = k_resultF;
    o[0].detail  = k_RixSCVarying;
    o[0].value = (RtPointer) resultF;

    RtColorRGB *resultRGB = NULL;
    resultRGB = pool.AllocForPattern<RtColorRGB>(sctx->numPts);
    o[1].paramId = k_resultRGB;
    o[1].detail  = k_RixSCVarying;
    o[1].value = (RtPointer) resultRGB;


    RtFloat *resultDispl = NULL;
    resultDispl = pool.AllocForPattern<RtFloat>(sctx->numPts);
    o[2].paramId = k_resultDispl;
    o[2].detail  = k_RixSCVarying;
    o[2].value = (RtPointer) resultDispl;




    // check for manifold input
    RixSCType type;
    RixSCConnectionInfo cinfo;

    RtPoint3 *sP;
    sctx->GetParamInfo(k_manifold, &type, &cinfo);
    if (cinfo != k_RixSCNetworkValue)
    {
        // We want P by default (not st)
        RtPoint3 const *Q;


      //  if (*surfacePosition == k_usePo)
            sctx->GetBuiltinVar(RixShadingContext::k_Po, &Q);
        //else
        //    sctx->GetBuiltinVar(RixShadingContext::k_P, &Q);

        sP = pool.AllocForPattern<RtPoint3>(sctx->numPts);
        memcpy(sP, Q, sizeof(RtPoint3)*sctx->numPts);

        // transform P in object space by default
        //
        sctx->Transform(RixShadingContext::k_AsPoints,
                        Rix::k_current, Rix::k_object, sP, NULL);
    }
    else
    {
        RtPoint3 const *mQ;
        sctx->EvalParam(k_manifoldQ, -1, &mQ, &m_defaultST, true);
        sP = const_cast<RtPoint3*>(mQ);
    }


    std::string input = "test_pointcloud";
    char* _output_file_name;
    _output_file_name = (char *) malloc((input.size()+1) * sizeof(char));
    input.copy(_output_file_name, input.size() + 1);
    _output_file_name[input.size()] = '\0';
    PtcPointCloud inptc = PtcSafeOpenPointCloudFile( _output_file_name);
        if (!inptc) {
         std::cout<<"Error";
         exit(1);
        }


        float scale =2* M_PI;
        float cell_scale = 1;
        float maxdist = cell_scale;
      //  scale = scale   / cell_scale;
        float *data;
        float point[3];
        float normal[3];
        int hardness_levels = 3;
        float hardness_values[hardness_levels];
        float max_hardness_value;
        hardness_values[0] = 23;
        hardness_values[1] = 28;
        hardness_values[2] = 35;
        max_hardness_value = 35;

        RtColorRGB hardness_colors[hardness_levels];

        hardness_colors[0].r = 0.17;
        hardness_colors[0].g = 0.13;
        hardness_colors[0].b = 0.02;


        hardness_colors[1].r = 0.19;
        hardness_colors[1].g = 0.12;
        hardness_colors[1].b = 0.08;

        hardness_colors[2].r = 0.16;
        hardness_colors[2].g = 0.1;
        hardness_colors[2].b = 0.07;

        int datasize;
        normal[0] = normal[1] = normal[2] = 0;

        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
        data = (float *) malloc(datasize * sizeof(float));
        float val1 =0;
        float val2 =0;
        float Readres = -1;
        float res = 0;
float proj = 0;

    RtPoint3 f1cell, f2cell,f3cell,f4cell,xcell,ycell,zcell;
    for (int n = 0; n < sctx->numPts; ++n)
    {
        float f1,f2,f3,f4,d;
         RtPoint3 pp =  sP[n];

         RtPoint3 testpoint = pp;
         RtPoint3 thiscell = RtPoint3 (cell_scale* (floorf(pp.x/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.y/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.z/cell_scale ) + 0.5f));

         RtPoint3 inter_points[8];
         float inter_results[8];
         RtVector3 dir(0,0,0);

    /*    int z = 0;
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,-cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,-cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,cell_scale*0.5,  -cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,cell_scale*0.5, -cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,-cell_scale*0.5,-cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,-cell_scale*0.5, -cell_scale*0.5);
        float temp_res;
              for(z = 0;z<8;z++)
              {

                  point[0] = inter_points[z].x;
                  point[1] = inter_points[z].y;
                  point[2] = inter_points[z].z;
                 temp_res = 0;
                  Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 1, data);
                  //val2 = data[1];
                  if(Readres==1)
                  {
                      //testpoint = thiscell - sP[n];
                      testpoint = sP[n];//inter_points[z];
                      dir.x = (data[2]);
                      dir.y = (data[3]);
                      dir.z = (data[4]);
                      Normalize(dir);
                      proj = Dot(dir,testpoint);
                      //dir.x *= proj;
                      //dir.y *= proj;
                      //dir.z *= proj;
                      //float x = proj;//Dot(dir,RightDir);
                      //Normalize(dir);
                      //float costheta =  Dot(dir,RightDir);
                      //float rprime = (x*(1-costheta))/costheta;
                       temp_res = sin(scale*proj);

                  }

                  inter_results[z] = temp_res;

              }


*/

             float x0; = pp.x >thiscell.x ? thiscell.x - cell_scale*0.5;
             float x1; = thiscell.x + cell_scale*0.5;
             float y0; = thiscell.y - cell_scale*0.5;
             float y1; = thiscell.y + cell_scale*0.5;
             float z0; = thiscell.z - cell_scale*0.5;
             float z1; = thiscell.z + cell_scale*0.5;
            if(pp.x>thiscell.x)
            {
                x0 = thiscell.x;
                x1 = thiscell.x+cell_scale*0.5;
                xcell = thiscell+(cell_scale,0,0);
            }
            else
            {

                x0 = thiscell.x - cell_scale*0.5;
                x1 = thiscell.x;
                xcell = thiscell-(cell_scale,0,0);
            }

            if(pp.y>thiscell.y)
            {
                y0 = thiscell.y;
                y1 = thiscell.y+cell_scale*0.5;
                ycell = thiscell+(0,cell_scale,0);
            }
            else
            {

                y0 = thiscell.y - cell_scale*0.5;
                y1 = thiscell.y;
                ycell = thiscell-(0,cell_scale,0);
            }


            if(pp.z>thiscell.z)
            {
                z0 = thiscell.z;
                z1 = thiscell.z+cell_scale*0.5;
                zcell = thiscell-(0,0,cell_scale);
            }
            else
            {

                z0 = thiscell.z - cell_scale*0.5;
                z1 = thiscell.z;
                zcell = thiscell-(0,0,cell_scale);
            }



testpoint = pp;
        proj = 0;

    float differential_step = 1;
        f1 = f2 = f3= f4=1000.0f;
        for (int i = -1;  i <= 1;  i += 1)
        {
            for (int j = -1;  j <= 1;  j += 1)
            {
                for (int k = -1;  k <= 1;  k += 1)
                {


                    RtPoint3 testcell = thiscell + RtVector3(cell_scale*i,cell_scale*j,cell_scale*k);
                    RtPoint3 pos = testcell;// + jitter[n] *(RtVector3(m_sFuncs->CellNoise(testcell)) - 0.5f);
                    RtVector3 offset = pos - pp;//*Fbm(pp,3,0.5);
                    float dist;
                    switch (distancemetric)
                    {
                        case k_euclidean:
                            dist = sqrtf(Dot(offset, offset));
                            break;
                        case k_euclideanSquared:
                            dist = Dot(offset, offset);
                            break;
                        case k_manhattan:
                            dist =  fabsf(offset.x) +
                                    fabsf(offset.y) +
                                    fabsf(offset.z);
                            break;
                        case k_chebyshev:
                            offset.x = fabsf(offset.x);
                            offset.y = fabsf(offset.y);
                            offset.z = fabsf(offset.z);
                            d = (offset.x>offset.y)? offset.x:offset.y;
                            dist = (offset.z>d)? offset.z:d;
                            break;
                        case k_minkowski:
                            dist = powf(powf(fabsf(offset.x), minkowskiExp)+
                                        powf(fabsf(offset.y), minkowskiExp)+
                                        powf(fabsf(offset.z), minkowskiExp),
                                        1.0f/minkowskiExp);
                            break;
                        default:
                            dist = Dot(offset, offset);
                            break;
                    }


                    if (dist < f1)
                    {
                        f4 = f3;
                        f3 = f2;
                        f2 = f1;
                        f1 = dist;
                        f4cell = f3cell;
                        f3cell = f2cell;
                        f2cell = f1cell;
                        f1cell = pos;
                    }
                    else
                        if (dist < f2)
                        {
                            f4 = f3;
                            f3 = f2;
                            f2 = dist;
                            f4cell = f3cell;
                            f3cell = f2cell;
                            f2cell = pos;
                        }
                        else
                        if(dist<f3)
                        {
                            f4 = f3;
                            f3 = dist;
                            f4cell = f3cell;
                            f3cell = pos;
                        }
                        else if(dist<f4)
                        {
                            f4=dist;
                            f4cell = pos;
                        }
                }
            }
        }
        resultF[n] = 1;
        float offset = 0;
        /*if (shape == k_thin)
        {
            f1 = sqrtf(f1);
            f2 = sqrtf(f2);
        }
        else if (shape == k_fat)
        {
            f1 = f1*f1;
            f2 = f2*f2;
        }

        // combine distances to create the pattern
        //
        resultF[n] = f2 * c2[n] + f1 * c1[n];

        // post-process
        //
        if (clamp != 0)
        {
            resultF[n] = RixClamp(resultF[n], 0.0f, 1.0f);
        }

        if (invert != 0)
        {
            resultF[n]= 1.f - resultF[n];
        }*/




            point[0] = f1cell.x;
            point[1] = f1cell.y;
            point[2] = f1cell.z;


            int numpoints = 1; //16
            Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, numpoints, data);
            val1 = data[1];


            //point[0] = sP[n].x;
            //point[1] = sP[n].y;
            //point[2] = sP[n].z;
            //
            //Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 16, data);

            RtVector3 dir1(0,0,0);
            RtVector3 dir2(0,0,0);
            RtVector3 dir3(0,0,0);
            RtVector3 dir4(0,0,0);
            RtVector3 RightDir(1,0,0);
            float res1,res2,res3 = 0;
                dir1[0] = dir1[1] = dir1[2] = 0;
                if(Readres==1)
                {
                    testpoint = f1cell - sP[n];
                    dir1.x = (data[2]);
                    dir1.y = (data[3]);
                    dir1.z = (data[4]);
                    Normalize(dir1);
                    proj = Dot(dir1,testpoint);
                    //dir.x *= proj;
                    //dir.y *= proj;
                    //dir.z *= proj;
                    //float x = proj;//Dot(dir,RightDir);
                    //Normalize(dir);
                    //float costheta =  Dot(dir,RightDir);
                    //float rprime = (x*(1-costheta))/costheta;                
                    res1= sin(scale*proj);
                }

                point[0] = f2cell.x;
                point[1] = f2cell.y;
                point[2] = f2cell.z;

                int Readres2 = PtcGetNearestPointsData (inptc, point, normal,maxdist, numpoints, data);
                val2 = data[1];
                if(Readres2==1)
                {
                    testpoint = f2cell - sP[n];
                    dir2.x = (data[2]);
                    dir2.y = (data[3]);
                    dir2.z = (data[4]);
                    Normalize(dir2);
                    proj = Dot(dir2,testpoint);
                    //dir.x *= proj;
                    //dir.y *= proj;
                    //dir.z *= proj;
                    //float x = proj;//Dot(dir,RightDir);
                    //Normalize(dir);
                    //float costheta =  Dot(dir,RightDir);
                    //float rprime = (x*(1-costheta))/costheta;
                 res2 = sin(scale*proj);
                }

                point[0] = f3cell.x;
                point[1] = f3cell.y;
                point[2] = f3cell.z;

                int Readres3 = PtcGetNearestPointsData (inptc, point, normal,maxdist, numpoints, data);
               // val2 = data[1];
                if(Readres3==1)
                {
                    testpoint = f3cell- sP[n];
                    dir3.x = (data[2]);
                    dir3.y = (data[3]);
                    dir3.z = (data[4]);
                    Normalize(dir3);
                    proj = Dot(dir3,testpoint);
                    res3 = sin(scale*proj);

                }



                point[0] = f4cell.x;
                point[1] = f4cell.y;
                point[2] = f4cell.z;

                int Readres4 = PtcGetNearestPointsData (inptc, point, normal,maxdist, numpoints, data);
                val2 = data[1];
                if(Readres4==1)
                {
                    testpoint = f4cell - sP[n];
                    dir4.x = (data[2]);
                    dir4.y = (data[3]);
                    dir4.z = (data[4]);
                    Normalize( dir4);
                    proj = Dot(dir4,testpoint);
                    //dir.x *= proj;
                    //dir.y *= proj;
                    //dir.z *= proj;
                    //float x = proj;//Dot(dir,RightDir);
                    //Normalize(dir);
                    //float costheta =  Dot(dir,RightDir);
                    //float rprime = (x*(1-costheta))/costheta;
                 res2 = sin(scale*proj);
                }





                float diff1 = Dot(dir1,dir2);
                float diff2 = Dot(dir1,dir3);
                float diff3 = Dot(dir2,dir3);
        float treshold = 10;



            temp_res  = (res1 * (1-f1/(f1+f2)) + res2 * (1-f2/(f1+f2)) );


                float temp = (res1 * (1-f1/(f1+f3)) + res3 * (1-f3/(f1+f3)) );
                float res_total = (temp*(1-f3/(f3+f2)) + temp_res *(1-f2/(f3+f2)));
                res_total = res1 * (1-f1/(f1+f2+f3)) + res2 * (1-f2/(f1+f2+f3)) + res3 *(1-f3/(f1+f2+f3));
               //res1 = RixSmoothStep(0,1,res1);
               //
               //res2 = RixSmoothStep(0,1,res2);
                if(diff1<cos(M_PI/treshold))
                 {

                        if(diff2>cos(M_PI/treshold))
                        {
                            res = temp;
                        }
                        else
                        {
                            res = res1*(1-(f1/sqrtf(2* (cell_scale/2)*(cell_scale/2))));
                            //res = res1;
                        }

                 }
                 else
                 {
                     //if(diff2<cos(M_PI/treshold))
                     //{
                     //    res = res;
                     //}
                     //else
                     {
                         res = temp_res;
                     }
                 }
                res = 0.5 + 0.5*res;
                float blend = RixSmoothStep(0.4, 0.8, res);

                             float xd = (pp.x - x0)/(x1-x0);
                            float yd = (pp.y - y0)/(y1-y0);
                            float zd = (pp.z - z0)/(z1-z0);




                            float c00 = inter_results[0]*(1-xd) + inter_results[1]*(xd);
                            float c01 = inter_results[3]*(1-xd) + inter_results[2]*(xd);
                            float c10 = inter_results[4]*(1-xd) + inter_results[5]*(xd);
                            float c11 = inter_results[7]*(1-xd) + inter_results[6]*(xd);
                            float cz0=  c00*(1-yd) + c01*(yd);
                            float cz1=  c10*(1-yd) + c11*(yd

/*
                           float c00 = inter_results[0]*(1-xd) + inter_results[1]*(xd);
                           float c01 = inter_results[3]*(1-xd) + inter_results[2]*(xd);
                           float c10 = inter_results[4]*(1-xd) + inter_results[5]*(xd);
                           float c11 = inter_results[7]*(1-xd) + inter_results[6]*(xd);
                           float cz0=  c00*(1-yd) + c01*(yd);
                           float cz1=  c10*(1-yd) + c11*(yd);
                            res = cz0*(1-zd) + cz1*(zd);
                            res = cz0;
*/
 //     Readres = PtcGetNearestPointsData (inptc, point, normal,1.5, 2, data1);
 //   float dir[3];


        float dist1,dist2 ;
        float min_dist1 = 10.0f;
        float min_dist2 = 10.0f;

        int index1 = -1;
        int index2 = -1;
       // res = roundf(res);
        for(int i = 0;i<hardness_levels;i++)
        {
            dist1 = val1-hardness_values[i];
            dist1 = dist1<0 ? -dist1 : dist1;

            dist2 = val2-hardness_values[i];
            dist2 = dist2<0 ? -dist2 : dist2;

            if(dist1<min_dist1)
            {
                min_dist1 = dist1;
                index1 = i;
            }

            if(dist2<min_dist2)
            {
                min_dist2 = dist2;
                index2 = i;
            }
        }
        float displ;
        if(index1>=0)
        {
            resultRGB[n].r = hardness_colors[index1].r;
            resultRGB[n].g = hardness_colors[index1].g;
            resultRGB[n].b = hardness_colors[index1].b;
            displ = val1;//hardness_values[index1]/max_hardness_value;
            displ = displ <= 1 ? displ : 1;
            resultDispl[n] = displ;

        }
        else
        {
            resultDispl[n] = 0;
           // resultF[n] = 0 ;
            resultRGB[n].r = 1;
            resultRGB[n].g = 0;
            resultRGB[n].b = 0;
        }

        if (randomScale[n] != 0.f &&  index1!=index2)
        {
            float mask = powf(f2-f1, 0.5);//0.25f);
            mask = RixSmoothStep(randomScaleCenter[n], 1.f, mask);
            float scale = (m_sFuncs->CellNoise(f1cell) * mask);
            scale = (scale - randomScaleCenter[n]) * (1.f/(1.f - randomScaleCenter[n]));
            if (c1[n] != 0.f || c2[n] != 0.f)
                resultDispl[n] *= RixMix(1.f, scale, randomScale[n]);
            else
                resultDispl[n] = RixMix(0.f, scale, randomScale[n]);
        }


        //if(val > 20.7 && val<25.3)
        //{

            RtColorRGB red(1,0,0);
            RtColorRGB white(1,1,1);
            resultRGB[n] = RixLerpRGB(white,red,blend);
            resultRGB[n].r = resultRGB[n].b = resultRGB[n].g =res;
            //if(res == 0)
            //{
            //    resultRGB[n].r = 1;//0.2;
            //    resultRGB[n].g = 1;//0.1;
            //    resultRGB[n].b = 0;//0.1;
            //}
            //else
            //{
            //
            //            resultRGB[n].r = 0;//0.2;
            //            resultRGB[n].g = 0;//0.1;
            //            resultRGB[n].b = 1;//0.1;
            //
            //}
        //    dist = 23-val;
        //    dist = dist/2.3;
        //    resultRGB[n].r = 0.7 + dist*0.1;
        //    resultRGB[n].g = 0.6 + dist*0.1;
        //    resultRGB[n].b = 0.5 + dist*0.1;
        //}
        //else
        //{
        //    if(val > 31.5 && val<38.5)
        //    {
        //       // if(res == 0)
        //       // {
        //       //     resultRGB[n].r = 1;//0.2;
        //       //     resultRGB[n].g = 0;//0.1;
        //       //     resultRGB[n].b = 0;//0.1;
        //       // }
        //       // else
        //       // {
        //       //
        //       //             resultRGB[n].r = 0;//0.2;
        //       //             resultRGB[n].g = 1;//0.1;
        //       //             resultRGB[n].b = 0;//0.1;
        //       //
        //       // }
        //        dist = 35-val;
        //        dist = dist/3.5;
        //        resultRGB[n].r = 0.6 + dist*0.1;
        //        resultRGB[n].g = 0.7 + dist*0.1;
        //        resultRGB[n].b = 0.3 + dist*0.1;
        //    }
        //    else
        //    {
        //        resultRGB[n].r = 0;//0.2;
        //        resultRGB[n].g = 0;//0.1;
        //        resultRGB[n].b = 0;//0.1;
        //    }
        //}

        //if(res == 0)
        //{
        //    resultRGB[n].r = 1;//0.2;
        //    resultRGB[n].g = 0;//0.1;
        //    resultRGB[n].b = 0;//0.1;
        //}
        //else
        //{
        //        if(res==1)
        //        {
        //            resultRGB[n].r = 0;//0.2;
        //            resultRGB[n].g = 1;//0.1;
        //            resultRGB[n].b = 0;//0.1;
        //        }
        //        else
        //        {
        //            resultRGB[n].r = 0;//0.2;
        //            resultRGB[n].g = 0;//0.1;
        //            resultRGB[n].b = 1;//0.1;
        //        }
        //
        //
        //}



 //   for (unsigned i=0; i<sctx->numPts; i++)
 //   {
 //      // resultRGB[i] = resultRGB[i] * colorScale[i] + colorOffset[i];
 //       resultF[i] = resultF[i] * floatScale[i] + floatOffset[i];
 //   }
}
    return 0;
}

RIX_PATTERNCREATE
{
    PIXAR_ARGUSED(hint);

    return new PxrWorleyD();
}

RIX_PATTERNDESTROY
{
    delete ((PxrWorleyD*)pattern);
}
