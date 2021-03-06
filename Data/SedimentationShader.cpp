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

    float DDA(RtPoint3 pp,RtVector3 dir,int _FlowLength,float scale);
    float LIC(RtPoint3 pp,PtcPointCloud inptc,float *data,int _FlowLength,float scale);
    RtVector3 FindOrthogonalVector(RtVector3 dir);
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

RtVector3 PxrWorleyD::FindOrthogonalVector(RtVector3 dir)
{
    RtVector3 UpDir(1,0,0);
    RtVector3 temp_vector = Cross(dir,UpDir);
    if(temp_vector[0]==0 && temp_vector[1]==0 && temp_vector[2]==0)
    {
        UpDir = RtVector3(0,1,0);
        temp_vector = Cross(dir,UpDir);
    }
    RtVector3 orthogonal_vec= Cross(dir,temp_vector);
    Normalize(orthogonal_vec);
    return orthogonal_vec ;
}

float PxrWorleyD::DDA(RtPoint3 pp,RtVector3 dir,int _FlowLength,float scale)
{
 int w = 0;
float step_size = scale > 1 ? 1/(scale*2) : scale;

float maxdist = scale*2;
float normal[3] =  {0,0,0};
float _st0[3],_st1[3];

int Readres = 0;
float col = m_sFuncs->Noise(pp*scale);
RtPoint3 st0 = pp;

 for(int i = 0; i < _FlowLength; i++) {

      st0 += dir*step_size;
     float n = m_sFuncs->Noise(st0*scale);
     col += n;
     w++;
 }



RtPoint3 st1 = pp;
for(int i = 0; i < _FlowLength; i++) {
    st1 -= dir*step_size;
    float n = m_sFuncs->Noise(st1*scale);
    col += n;
    w++;
 }

 col /= w;

return col ;
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
       //     sctx->GetBuiltinVar(RixShadingContext::k_P, &Q);

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


    std::string input = "pointcloud_SedimentationShader";
    char* _output_file_name;
    _output_file_name = (char *) malloc((input.size()+1) * sizeof(char));
    input.copy(_output_file_name, input.size() + 1);
    _output_file_name[input.size()] = '\0';
    PtcPointCloud inptc = PtcSafeOpenPointCloudFile( _output_file_name);
        if (!inptc) {
         std::cout<<"Error";
         exit(1);
        }

        float scaler = 4;
        float scale =2 * M_PI/scaler;
        float cell_scale = 1;
        //cell_scale;
      //  scale = scale   / cell_scale;
        float *data;
        float point[3];
        float normal[3];
        int hardness_levels = 9;
        float hardness_values[hardness_levels];
        float max_hardness_value;
        hardness_values[0] = 23;
        hardness_values[1] = 28;
        hardness_values[2] = 35;
        max_hardness_value = 35;

        RtColorRGB hardness_colors[hardness_levels];

        hardness_colors[0].r =0;//0.18;//0;// 0.3456;//
        hardness_colors[0].g =0;//0.13;//0;// 0.3456;//
        hardness_colors[0].b =0;//0.09;//0;// 0.3456;//

        float val_1,val_2,val_3;
        val_1 =180/(float)255;
        val_2 =167/(float)255;
        val_3 =141/(float)255;

        hardness_colors[1].r = val_1;//1;//0.219;
        hardness_colors[1].g = val_2;//0;//0.112;
        hardness_colors[1].b = val_3;//1;//0.074;

        val_1 =137/(float)255;
        val_2 =127/(float)255;
        val_3 =107/(float)255; //
        hardness_colors[2].r = val_1;//1; //0.16;
        hardness_colors[2].g = val_2;//0;//0.13;
        hardness_colors[2].b = val_3;//0; //0.07;

        val_1 =169/(float)255;
        val_2 =153/(float)255;
        val_3 =121/(float)255; //
        hardness_colors[3].r = val_1;//0; //0.17;
        hardness_colors[3].g = val_2;//1;//0.11;
        hardness_colors[3].b = val_3;//1; //0.09;

        val_1 =147/(float)255;
        val_2 =115/(float)255;
        val_3 =55/(float)255;//
        hardness_colors[4].r = val_1;//1; //0.15;
        hardness_colors[4].g = val_2;//1; //0.9;
        hardness_colors[4].b = val_3;//0;//0.075;
        val_1 =168/(float)255;
        val_2 =120/(float)255;
        val_3 =81/(float)255;


                                  //
        hardness_colors[5].r = val_1;//0; //0.15;
        hardness_colors[5].g = val_2;//0;//0.13;
        hardness_colors[5].b = val_3;//1; //0.07;
        val_1 =128/(float)255;
        val_2 =87/(float)255;
        val_3 =54/(float)255;//
                                 //
        hardness_colors[6].r = val_1;//0; //0.16;
        hardness_colors[6].g = val_2;//1;//0.16;
        hardness_colors[6].b = val_3;//0; //0.09;
         //
        val_1 =110/(float)255;
        val_2 =86/(float)255;
        val_3 =40/(float)255;//              //
        hardness_colors[7].r = val_1;//0.7; //0.16;
        hardness_colors[7].g = val_2;//0.3; //0.1;
        hardness_colors[7].b = val_3;//0.7; //0.02;
        val_1 =176/(float)255;
        val_2 =144/(float)255;
        val_3 =84/(float)255;//

        hardness_colors[8].r = val_1;//0.3; //0.16;
        hardness_colors[8].g = val_2;//0.7; //0.1;
        hardness_colors[8].b = val_3;//0.7; //0.02;


        RtColorRGB hardness_colors_secondary[hardness_levels];


        hardness_colors_secondary[0].r = 0.18*1.5;
        hardness_colors_secondary[0].g = 0.13*1.5;
        hardness_colors_secondary[0].b = 0.09*1.5;
        hardness_colors_secondary[1].r = 0.19*1.5;
        hardness_colors_secondary[1].g = 0.12*1.5;
        hardness_colors_secondary[1].b = 0.08*1.5;
        hardness_colors_secondary[2].r = 0.16*1.5;
        hardness_colors_secondary[2].g =  0.1*1.5;
        hardness_colors_secondary[2].b = 0.07*1.5;

       // hardness_colors_secondary[0].r = 0.21;
       // hardness_colors_secondary[0].g = 0.16;
       // hardness_colors_secondary[0].b = 0.13;
       // hardness_colors_secondary[1].r = 0.24;
       // hardness_colors_secondary[1].g = 0.17;
       // hardness_colors_secondary[1].b = 0.11;
       // hardness_colors_secondary[2].r = 0.19;
       // hardness_colors_secondary[2].g = 0.17;
       // hardness_colors_secondary[2].b =  0.1;


        int datasize;
        normal[0] = normal[1] = normal[2] = 0;

        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
        data = (float *) malloc(datasize * sizeof(float));
        float val1 = 0;
        float val2 =0;
        float Readres = -1;
        float res = 0;

    RtPoint3 f1cell, f2cell;
    for (int n = 0; n < sctx->numPts; ++n)
    {
         RtPoint3 pp =  sP[n];

/*        float f1,f2,f3,f4,d;
        float nois = m_sFuncs->Noise(scaler*sP[n]/scale);
        nois = 0;

         RtPoint3 testpoint = pp;
         RtPoint3 thiscell = RtPoint3 (cell_scale* (floorf(pp.x/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.y/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.z/cell_scale ) + 0.5f));


         f1 = f2 = 1000.0f;
         for (int i = -1;  i <= 1;  i += 1)
         {
             for (int j = -1;  j <= 1;  j += 1)
             {
                 for (int k = -1;  k <= 1;  k += 1)
                 {
                     RtPoint3 testcell = thiscell + RtVector3(i,j,k);
                     RtPoint3 pos = testcell + jitter[n] *
                                    (RtVector3(m_sFuncs->CellNoise(testcell)) - 0.5f);
                     RtVector3 offset = pos - pp;
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
                         f2 = f1;
                         f1 = dist;
                         f2cell = f1cell;
                         f1cell = pos;
                     }
                     else if (dist < f2)
                     {
                         f2 = dist;
                         f2cell = pos;
                     }
                 }
             }
         }
*/

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
std::map<int,int> used_id;
         //std::<int> used_id;
        resultF[n] = 1;
       float offset = 0;

                int K = 1;
                float maxdist = 1.25;
               //point[0]= f1cell.x;
               //point[1]= f1cell.y;
               //point[2]= f1cell.z;
                point[0] = pp.x;//thiscell.x;
                point[1] = pp.y;//thiscell.y;
                point[2] = pp.z;//thiscell.z;
                int Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
                float val = 0;


                if(Readres==1)
                {
                   val= data[0];
                  // RtPoint3 pp = sP[n];
                   /*RtPoint3 max,min;
                   {

                         RtPoint3 thiscell = RtPoint3 (cell_scale* (floorf(pp.x/cell_scale ) ),
                                                     cell_scale* (floorf(pp.y/cell_scale ) ),
                                                     cell_scale* (floorf(pp.z/cell_scale ) ));
                       if(thiscell.x<pp.x){
                           min.x = thiscell.x;
                           max.x = thiscell.x+1*cell_scale;
                       }else{
                           max.x = thiscell.x;
                           min.x = thiscell.x-1*cell_scale;
                       }
                       if(thiscell.y<pp.y){
                           min.y = thiscell.y;
                           max.y = thiscell.y+1*cell_scale;
                       }else{
                           max.y = thiscell.y;
                           min.y = thiscell.y-1*cell_scale;
                       }
                       if(thiscell.z<pp.z){
                           min.z = thiscell.z;
                           max.z = thiscell.z+1*cell_scale;
                       }else{
                           max.z = thiscell.z;
                           min.z = thiscell.z-1*cell_scale;
                       }
                    //   std::cout<<cell_scale<<std::endl;
                        RtPoint3 Xdir(cell_scale,0,0);
                        RtPoint3 Ydir(0,cell_scale,0);
                        RtPoint3 Zdir(0,0,cell_scale);
                       float fractX = (pp.x-min.x)/cell_scale;
                       float fractY = (pp.y-min.y)/cell_scale;
                       float fractZ = (pp.z-min.z)/cell_scale;
                       float values[4];
                       RtPoint3 interp_points[4];
                       interp_points[0] = min;
                       interp_points[1] = min + Xdir;
                       interp_points[2] = min + Ydir;
                       interp_points[3] = min + Ydir + Xdir;
                      // interp_points[4] = min + Zdir;
                      // interp_points[5] = min + Zdir + Xdir ;
                      // interp_points[6] = min + Zdir + Ydir;
                      // interp_points[7] = min + Zdir + Ydir + Xdir;
                    bool skip = false;
                       for(int j= 0;j<4;j++){

                           point[0] = interp_points[j][0];
                           point[1] = interp_points[j][1];
                           point[2] = interp_points[j][2];
                   //    std::cout<<"point "<<point[0]<<std::endl;}
                        Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist/2, K, data);
                        if(Readres==1){
                            values[j] = data[0];
                        }else{
                            values[j] = 0;
                        }


                       }
                       for(int j = 0;j<4;j++){
                           int temp = values[j];
                           if(used_id.count(temp)>0){
                               used_id[temp]++;
                           }else{
                               used_id.insert(std::make_pair(temp,1));
                           }
                       }
                       int max = 0;
                       int final_id = 0;
                       for(auto const entry:used_id){
                           if(entry.second>0){
                               max = entry.second;
                               final_id = entry.first;
                           }
                       }
                       val = final_id;*/
/*
                           float X0 =     RixMix(values[0], values[1], fractX);
                           float X1 =     RixMix(values[2], values[3], fractX);
                           float X2 =     RixMix(values[4], values[5], fractX);
                           float X3 =     RixMix(values[6], values[7], fractX);
        float Y0 =           RixMix(X0,X1, fractY);
        float Y1 =           RixMix(X2,X3, fractY);
        float Z = RixMix(Y0,Y1, fractZ);
        //std::cout<<"X0 "<<X0<<std::endl;
        //std::cout<<fractX<<" "<<fractY<<" "<<fractZ <<std::endl;
        val = Z;*/
            //      }
                }




/*
                float delta = 1;
                maxdist = delta*0.5;
                point[0] = pp.x + delta;//thiscell.x;
                point[1] = pp.y;//thiscell.y;
                point[2] = pp.z;//thiscell.z;
                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
                float dx_val = 0;

                if(Readres==1)
                {

                    dx_val = data[0];
                }


                point[0] = pp.x - delta;//thiscell.x;
                point[1] = pp.y;//thiscell.y;
                point[2] = pp.z;//thiscell.z;
                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
                float dx_minus_val = 0;

                if(Readres==1)
                {

                    dx_minus_val = data[0];
                }



                point[0] = pp.x;//thiscell.x;
                point[1] = pp.y + delta;//thiscell.y;
                point[2] = pp.z;//thiscell.z;
                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
                float dy_val = 0;

                if(Readres==1)
                {

                    dy_val = data[0];
                }

                point[0] = pp.x;//thiscell.x;
                point[1] = pp.y - delta;//thiscell.y;
                point[2] = pp.z;//thiscell.z;
                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
                float dy_minus_val = 0;

                if(Readres==1)
                {

                    dy_minus_val = data[0];
                }

                //res = val;
                res = val*0.5 + dy_minus_val * 0.125+ dy_val*0.125 + dx_val*0.125 + dx_minus_val*0.125;*/
                //res = dy_minus_val * 0.25+ dy_val*0.25 + dx_val*0.25 + dx_minus_val*0.25;
                /*
                RtVector3 dir_details = dir;
                dir = FindOrthogonalVector(dir);

                point[0] = f1cell.x;
                point[1] = f1cell.y;
                point[2] = f1cell.z;


                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 1, data);


                if(Readres==1)
                {
                    val1 = data[0];

                }

                point[0] = f2cell.x;
                point[1] = f2cell.y;
                point[2] = f2cell.z;

                Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 1, data);


                if(Readres==1)
                {
                  //  testpoint = thiscell - pp;
                  val2 = data[0];
                }
*/




//res = details;

res = RixSmoothStep(0,1 ,res );
/*float mock_scale = 0.1;
int round_z = roundf(pp.z);
int round_x = roundf(pp.x);
float a = -0.1;
float b = -1.0f;
float val_1 = pp.y*a+b;
float index =0;
if(val_1<=pp.z)
    index = 1;
else{
    a= -0.3;
    b=1.5;
    val_1 = pp.y*a+b;
    if(val_1<=pp.z)
        index = 2;
    else
        index = 3;
}*/

//index++;

//float blend = RixSmoothStep(0,1 ,res );

/*
        float dist1,dist2 ;
        float min_dist1 = 10.0f;
        float min_dist2 = 10.0f;

        int index1 = -1;
        int index2 = -1;

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
*/

RtColorRGB  col,col1,col2 ;
int index1 = roundf(val);

//offset = std::sin(pp[1]);
//int z_height =ceilf(pp[2]+offset);
//index1 = z_height%3;
//index1 = index1 % 3;
//if(index1==0)
//    index1 = 3;

/*
int index2 = roundf(dx_val);
int index3 = roundf(dx_minus_val);
int index4 = roundf(dy_val);
int index5 = roundf(dy_minus_val);*/
//col = hardness_colors[index1];

float displ =0 ;
        if(index1>=0 && index1<hardness_levels)
        {
        //    std::cout<<"sedimentation"<<std::endl;
            col = hardness_colors[index1];
            //col = hardness_colors[val]*0.5 + hardness_colors[dx_val]*0.125 + hardness_colors[dx_minus_val]*0.125 + hardness_colors[dy_val]*0.125 + hardness_colors[dy_minus_val]*0.125

            //col = hardness_colors[index2];
      /*  col = RixLerpRGB(hardness_colors[index2],hardness_colors[index3], 0.5);
        col1 = RixLerpRGB(hardness_colors[index4],hardness_colors[index5], 0.5);
        col2 = RixLerpRGB(col,col1, 0.5);
        col = RixLerpRGB(col2,hardness_colors[index1],0.5);*/
        //  resultRGB[n].r = hardness_colors[index1].r;
          //  resultRGB[n].g = hardness_colors[index1].g;
          //  resultRGB[n].b = hardness_colors[index1].b;

           // displ = hardness_values[index1]/max_hardness_value;//max_hardness_value;
           // if(index2!= index1)
           // {
           //     displ =  (f1/(f1+f2))*hardness_values[index1]/max_hardness_value +  (f2/(f1+f2))*hardness_values[index2]/max_hardness_value;;
           // }
         //   displ = displ <= 1 ? displ : 1;

        }
        else
        {
                       // resultF[n] = 0 ;
            displ = 0;
            col.r = 0;
            col.g = 0;
            col.b = 0;
        }
/*
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
*/




            RtColorRGB black(1,1,1);
            RtColorRGB white(1,1,1);
            //resultRGB[n] = RixLerpRGB(black,white,blend);
            //if(index1 != -1)
            float center = 0.12;
            offset = 0.1;

          //resultRGB[n] = RixLerpRGB( red,col,red_stripes);
          //resultRGB[n] = RixLerpRGB( green,resultRGB[n],green_stripes);

        //resultRGB[n] = RixLerpRGB( col,hardness_colors_secondary[index1],color_details);

        //resultRGB[n] = hardness_colors_secondary[index1];
        //                resultRGB[n] = RixLerpRGB( col,hardness_colors_secondary[index1],color_details);
        //else
        //             resultRGB[n]  = RtColorRGB(0.3,0.3,0.3);
           // resultRGB[n] = RixLerpRGB(red, blue,res);
          //   resultRGB[n] = RixLerpRGB( hardness_colors[0], hardness_colors[1],res);
            // res = 0.5*Fbm(pp*0.4,4,0.5);

             //res = res + 0.5*Fbm(pp*0.4,4,0.3);

          //   resultRGB[n].r = resultRGB[n].b = resultRGB[n].g =res;
            resultRGB[n] = col;
    //resultRGB[n] = RtColorRGB(1,0,0);
          resultF[n] = 0;
                  //(1-res)*displ_mult + displ;
          //resultDispl[n] = 0;//  displ;


}

   /* for (unsigned i=0; i<sctx->numPts; i++)
    {
        resultRGB[i] = resultRGB[i] * colorScale[i] + colorOffset[i];
        resultF[i] = resultF[i] * floatScale[i] + floatOffset[i];
    }*/
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

