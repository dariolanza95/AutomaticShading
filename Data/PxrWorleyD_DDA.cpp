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

float PxrWorleyD::LIC(RtPoint3 pp,PtcPointCloud inptc,float *data,int _FlowLength,float scale)
{
 int w = 0;
float step_size = scale > 1 ? 1/(scale*2) : scale;

float maxdist = scale*2;
float normal[3] =  {0,0,0};
float _st0[3],_st1[3];
RtVector3 dir = RtVector3(0,0,0);
int Readres = 0;
float col = m_sFuncs->Noise(pp*scale);
RtPoint3 st0 = pp;
// float _st0[3];
// float _st1[3];
_st0[0] = st0.x;
_st0[1] = st0.y;
_st0[2] = st0.z;
Readres = PtcGetNearestPointsData (inptc, _st0, normal,maxdist, 4, data);

if(Readres==1)
{
   dir.x = (data[2]);
   dir.y = (data[3]);
   dir.z = (data[4]);
   Normalize(dir);
}


 for(int i = 0; i < _FlowLength; i++) {

      st0 += dir*step_size;
     float n = m_sFuncs->Noise(st0*scale);

     _st0[0] = st0.x;
     _st0[1] = st0.y;
     _st0[2] = st0.z;
     Readres = PtcGetNearestPointsData (inptc, _st0, normal,maxdist, 4, data);

    if(Readres==1)
    {
        dir.x = (data[2]);
        dir.y = (data[3]);
        dir.z = (data[4]);
        Normalize(dir);
    }

     col += n;
     w++;
 }

 _st0[0] = pp.x;
 _st0[1] = pp.y;
 _st0[2] = pp.z;

 Readres = PtcGetNearestPointsData (inptc, _st0, normal,maxdist, 4, data);

 if(Readres==1)
 {
     dir.x = (data[2]);
     dir.y = (data[3]);
     dir.z = (data[4]);
    Normalize(dir);
 }

RtPoint3 st1 = pp;
for(int i = 0; i < _FlowLength; i++) {
   st1 -= dir*step_size;
     float n = m_sFuncs->Noise(st1*scale);

       _st1[0] = st1.x;
       _st1[1] = st1.y;
       _st1[2] = st1.z;
       Readres = PtcGetNearestPointsData (inptc, _st1, normal,maxdist, 4, data);

      if(Readres==1)
      {
          dir.x = (data[2]);
          dir.y = (data[3]);
          dir.z = (data[4]);
          Normalize(dir);
      }



         col += n;

         w++;
 }

 col /= w;

return col ;
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
        //    sctx->GetBuiltinVar(RixShadingContext::k_Po, &Q);
        //else
            sctx->GetBuiltinVar(RixShadingContext::k_P, &Q);

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

        float scaler = 4;
        float scale =2 * M_PI/scaler;
        float cell_scale = 1;
        float maxdist = 10;//cell_scale;
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

        hardness_colors[0].r = 0.18;
        hardness_colors[0].g = 0.13;
        hardness_colors[0].b = 0.09;


        hardness_colors[1].r = 0.19;
        hardness_colors[1].g = 0.12;
        hardness_colors[1].b = 0.08;

        hardness_colors[2].r = 0.16;
        hardness_colors[2].g = 0.1;
        hardness_colors[2].b = 0.07;


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

        int datasize;
        normal[0] = normal[1] = normal[2] = 0;

        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
        data = (float *) malloc(datasize * sizeof(float));
        float val1 = 0;
        float val2 =0;
        float Readres = -1;
        float res = 0;
float proj = 0;

    RtPoint3 f1cell, f2cell,f3cell,f4cell,xcell_l,xcell_r,ycell_up,ycell_bot,zcell,xycell,zxcell,zycell,zxycell;
    for (int n = 0; n < sctx->numPts; ++n)
    {
        float f1,f2,f3,f4,d;
        float nois = m_sFuncs->Noise(scaler*sP[n]/scale);
        nois = 0;
         RtPoint3 pp =  sP[n];//+ 0.005*nois*sP[n];

         RtPoint3 testpoint = pp;
         RtPoint3 temppoint;
         RtPoint3 thiscell = RtPoint3 (cell_scale* (floorf(pp.x/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.y/cell_scale ) + 0.5f),
                                       cell_scale* (floorf(pp.z/cell_scale ) + 0.5f));


         int cell_disparity = floorf(pp.x/cell_scale)+floorf(pp.y/cell_scale) + floorf(pp.z/cell_scale);
         bool  IsInterpolantCell = cell_disparity % 2 == 0 ? true: false;
         RtPoint3 inter_points[8];
         float inter_results[8];

float temp_res;
        int z = 0;
             /*
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,-cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,-cell_scale*0.5,cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,cell_scale*0.5,  -cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,cell_scale*0.5, -cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(-cell_scale*0.5,-cell_scale*0.5,-cell_scale*0.5);
              inter_points[z++] = thiscell - RtPoint3(cell_scale*0.5,-cell_scale*0.5, -cell_scale*0.5);
        float temp_dir;





             float x0 = thiscell.x - cell_scale*0.5;
             float x1 = thiscell.x + cell_scale*0.5;
             float y0 = thiscell.y - cell_scale*0.5;
             float y1 = thiscell.y + cell_scale*0.5;
             float z0 = thiscell.z - cell_scale*0.5;
             float z1 = thiscell.z + cell_scale*0.5;

             xcell_l = thiscell;
             ycell_up = thiscell;
             zcell = thiscell;
             xycell = thiscell;
             zxcell = thiscell;
             zycell = thiscell;
             zxycell = thiscell;

           /*if(pp.x>thiscell.x)
            {
                x0 = thiscell.x;
                x1 = thiscell.x+cell_scale*0.5;
                xcell_l.x += cell_scale;
                xycell.x += cell_scale;
                zxcell.x += cell_scale;
                zxycell.x += cell_scale;
            }
            else
            {

                x0 = thiscell.x - cell_scale*0.5;
                x1 = thiscell.x;
                xcell_l.x -= cell_scale;
                xycell.x -= cell_scale;
                zxcell.x  -= cell_scale;
                zxycell.x -= cell_scale;

            }*/
             xcell_l = xcell_r = thiscell;
             xcell_l.x -= cell_scale;
             xcell_r.x += cell_scale;

             ycell_up = ycell_bot = thiscell;
             ycell_up.y += cell_scale;
             ycell_bot.y -= cell_scale;



/*
            if(pp.y>thiscell.y)
            {
                y0 = thiscell.y;
                y1 = thiscell.y+cell_scale*0.5;
                //ycell = thiscell;//+(0,cell_scale,0);
                ycell_up.y += cell_scale;
                xycell.y += cell_scale;
                zycell.y += cell_scale;
               zxycell.y += cell_scale;

            }
            else
            {

                y0 = thiscell.y - cell_scale*0.5;
                y1 = thiscell.y;
                //ycell = thiscell;//-(0,cell_scale,0);
                ycell_up.y -= cell_scale;
                xycell.y -= cell_scale;
                zycell.y -= cell_scale;
               zxycell.y -= cell_scale;

            }


            if(pp.z>thiscell.z)
            {
                z0 = thiscell.z;
                z1 = thiscell.z+cell_scale*0.5;
                zcell = thiscell;//+(0,0,cell_scale);
                zcell.z += cell_scale;
                zycell.z += cell_scale;
                zxcell.z += cell_scale;
               zxycell.z += cell_scale;

            }
            else
            {

                z0 = thiscell.z - cell_scale*0.5;
                z1 = thiscell.z;
                zcell = thiscell;//-(0,0,cell_scale);
                zcell.z -= cell_scale;
                zycell.z -= cell_scale;
                zxcell.z -= cell_scale;
               zxycell.z -= cell_scale;
            }

      /*  inter_points[z++] = thiscell;
        inter_points[z++] = xcell;
        inter_points[z++] = ycell;
        inter_points[z++] = xycell;
        inter_points[z++] = zcell;
        inter_points[z++] = zxcell;
        inter_points[z++] = zycell;
        inter_points[z++] = zxycell;


        float temp_dir;
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
                testpoint = thiscell - pp;
               // testpoint = sP[n];//inter_points[z];
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
                 temp_dir = sin(scale*proj);

            }

            inter_results[z] = temp_dir;

        }
*/

testpoint = pp;
        proj = 0;

         f1 = f2 = f3= f4=1000.0f;
        for (int i = -1;  i <= 1;  i += 1)
        {
            for (int j = -1;  j <= 1;  j += 1)
            {
                for (int k = -1;  k <= 1;  k += 1)
                {


                    RtPoint3 testcell = thiscell + RtVector3(cell_scale*i,cell_scale*j,cell_scale*k);
                    RtPoint3 pos = testcell + jitter[n]* cell_scale*(RtVector3(m_sFuncs->CellNoise(testcell)) - 0.5f);
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







            int numpoints = 1; //16


            //point[0] = sP[n].x;
            //point[1] = sP[n].y;
            //point[2] = sP[n].z;
            //

            RtVector3 dir(0,0,0);

                point[0] = pp.x;//thiscell.x;
                point[1] = pp.y;//thiscell.y;
                point[2] = pp.z;//thiscell.z;

                int Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 27, data);
                float val = 1;

                if(Readres==1)
                {



                    dir.x = (data[2]);
                    dir.y = (data[3]);
                    dir.z = (data[4]);
                    //Normalize( dir);

                }


                point[0] = f1cell.x;
                point[1] = f1cell.y;
                point[2] = f1cell.z;


                 Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 1, data);


                if(Readres==1)
                {
                    val1 = data[1];

                }

                point[0] = f2cell.x;
                point[1] = f2cell.y;
                point[2] = f2cell.z;

                Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, 1, data);


                if(Readres==1)
                {
                  //  testpoint = thiscell - pp;
                  val2 = data[1];

                }





                     //   float3 col = tex2D(_NoiseTex, IN.uv);
/*
                        int w = 0;
                int _FlowLength = 5;

                //low frequencies
                float freq = 0.8;//0.8;
                //freq = Fbm(pp*0.2,6,0.8);
          //      freq = freq<0 ? 0.1 : freq;
                float step_size = freq > 1 ? 1/(freq*2) : freq;
                //step_size = 1/(freq*2);
                float weight_1[_FlowLength];
                float weight_2[_FlowLength];
                float previous_weight_1=0;
                float previous_weight_2=0;
                for(int i = 0;i<_FlowLength;i++) weight_1[i] = weight_2[i] = i;
            //    pp *= freq;

                float col = m_sFuncs->Noise(pp*freq);
                        RtPoint3 st0 = pp;
                        float _st0[3];
                        float _st1[3];
                        for(int i = 0; i < _FlowLength; i++) {

                             st0 += dir4*step_size;
                            float n = m_sFuncs->Noise(st0*freq);

                          //   n = n <= 0.5 ? 0 : 1;
                        //    n = n * (weight_1[i] - previous_weight_1);
                            previous_weight_1 = weight_1[i];



                            _st0[0] = st0.x;
                            _st0[1] = st0.y;
                            _st0[2] = st0.z;
                            Readres4 = PtcGetNearestPointsData (inptc, _st0, normal,maxdist, 4, data);

                           if(Readres4==1)
                           {
                               dir4.x = (data[2]);
                               dir4.y = (data[3]);
                               dir4.z = (data[4]);
                               Normalize(dir4);
                                }


                       //     n = sin(pp.x);
                            col += n;
                          //  col += 0.5*(1-i/_FlowLength)*n;
                            w++;
                        }

                        _st0[0] = pp.x;
                        _st0[1] = pp.y;
                        _st0[2] = pp.z;
                        Readres4 = PtcGetNearestPointsData (inptc, _st0, normal,maxdist, 4, data);

                        if(Readres4==1)
                        {
                            dir4.x = (data[2]);
                            dir4.y = (data[3]);
                            dir4.z = (data[4]);Normalize(dir4);

                             }
                       // float2 st1 = IN.uv;
                        RtPoint3 st1 = pp;
                     for(int i = 0; i < _FlowLength; i++) {
                          st1 -= dir4*step_size;
                            float n = m_sFuncs->Noise(st1*freq);
                                               //        n = n <= 0.5 ? 0 : 1;
                      //     n = n * (weight_2[i] - previous_weight_2);
                            previous_weight_2 = weight_2[i];


                        _st1[0] = st1.x;
                        _st1[1] = st1.y;
                        _st1[2] = st1.z;
                        Readres4 = PtcGetNearestPointsData (inptc, _st1, normal,maxdist, 4, data);

                       if(Readres4==1)
                       {
                           dir4.x = (data[2]);
                           dir4.y = (data[3]);
                           dir4.z = (data[4]);
                           Normalize(dir4);
                       }



                                col += n;

                            //    col += 0.5*(1-i/_FlowLength)*n;
                                w++;
                        }

                        col /= w;


                res = col;
            //    res = 1-res;
//                res = std::pow(2.0f, -8.0*res*res);

*/

 float displ_mult = 1;
    float dda= DDA(pp,dir,30,0.3);

    float lic = DDA(pp,dir,40,1.3);//LIC(pp,inptc,data,3,1.5);
    float details = DDA(pp,dir,40,3);
    float a  = 0;
    float b = 1;
    float c = 0.44;
    d = 0.56;
    lic = (lic - c)*((b-a)/(d-c)) + a;
    dda = (dda - c)*((b-a)/(d-c)) + a;
    c = 0.47;
    d = 0.53;
    details = (details - c)*((b-a)/(d-c)) + a;

    res = lic * 0.4 + dda*0.6;
    res = RixSmoothStep(0,1,res);
    res = RixSmoothStep(0,1,details);





res = RixSmoothStep(0,1 ,res );



float blend = RixSmoothStep(0,1 ,res );


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



        //index1 = min_dist1<min_dist2 ? index1 : index2;
        RtColorRGB col;
        float displ =0 ;
        if(index1>=0)
        {

            col = hardness_colors[index1];
          //  resultRGB[n].r = hardness_colors[index1].r;
          //  resultRGB[n].g = hardness_colors[index1].g;
          //  resultRGB[n].b = hardness_colors[index1].b;

            displ = hardness_values[index1]/max_hardness_value;//max_hardness_value;
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
            resultRGB[n].r = 1;
            resultRGB[n].g = 0;
            resultRGB[n].b = 0;
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


            RtColorRGB red(1,0,0);
            RtColorRGB blue(0,0,1);

            RtColorRGB black(1,1,1);
            RtColorRGB white(1,1,1);
            //resultRGB[n] = RixLerpRGB(black,white,blend);
            //if(index1 != -1)
                resultRGB[n] = RixLerpRGB( col,hardness_colors_secondary[index1],details);
            //else
                resultRGB[n]  = RtColorRGB(0.3,0.3,0.3);
         //   resultRGB[n] = RixLerpRGB( hardness_colors[0], hardness_colors[1],res);
             res = 0.5*Fbm(pp*0.4,4,0.5);

             //res = 0.5 + 0.5*Fbm(pp*0.4,4,0.3);

//             resultRGB[n].r = resultRGB[n].b = resultRGB[n].g =res;

          resultDispl[n] = (1-res)*displ_mult + displ;
          //resultDispl[n] = res;//  displ;

             //resultDispl[n] =  displ;

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

