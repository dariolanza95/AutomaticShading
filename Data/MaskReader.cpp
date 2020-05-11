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

#include "RixColorUtils.h"
#include <cstring>
#include "pointcloud.h"

class PxrTexture : public RixPattern
{
public:

    PxrTexture();
    virtual ~PxrTexture();

    virtual int Init(RixContext &, RtUString const pluginpath) override;
    virtual RixSCParamInfo const *GetParamTable() override;
    virtual void Synchronize(
        RixContext&,
        RixSCSyncMsg,
        RixParameterList const*) override
    {
    }

    virtual void Finalize(RixContext &) override;

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
    RtInt const m_firstChannel;
    RtInt const m_atlasStyle;
    RtInt const m_invertT;
    RtInt const m_filter;
    RtInt const m_lerp;
    RtColorRGB const m_missingColor;
    RtFloat const m_missingAlpha;
    RtInt const m_linearize;
    RtInt const m_mipBias;
    RtFloat const m_maxResolution;
    RtInt const m_optimizeIndirect;
    RtColorRGB const m_colorScale;
    RtColorRGB const m_colorOffset;
    RtFloat const m_alphaScale;
    RtFloat const m_alphaOffset;
    RtFloat const m_saturation;

    RixTexture *m_tex;
    RixMessages *m_msg;
};

PxrTexture::PxrTexture()
    : m_firstChannel(0),
      m_atlasStyle(RixTexture::AtlasNone),
      m_invertT(1),
      m_filter(RixTexture::TxParams::Box),
      m_lerp(1),
      m_missingColor(RtColorRGB(1.f, 0.f, 1.f)),
      m_missingAlpha(1.f),
      m_linearize(0),
      m_mipBias(0),
      m_maxResolution(0.f),
      m_optimizeIndirect(1),
      m_colorScale(1.f, 1.f, 1.f),
      m_colorOffset(0.f, 0.f, 0.f),
      m_alphaScale(1.f),
      m_alphaOffset(0.f),
      m_saturation(1.f),
      m_tex(NULL),
      m_msg(NULL)
{
}

PxrTexture::~PxrTexture()
{
}

int
PxrTexture::Init(RixContext &ctx, RtUString pluginpath)
{
    PIXAR_ARGUSED(pluginpath);

    m_tex = (RixTexture*)ctx.GetRixInterface(k_RixTexture);
    m_msg = (RixMessages*)ctx.GetRixInterface(k_RixMessages);

    /*RtUString const* filename = NULL;
    sctx->EvalParam(k_filename, -1, &filename);
    if (!filename)
    {
        m_msg->Error("PxrTexture failed: filename missing");
        return 1;
    }
    PtcPointCloud inptc = PtcSafeOpenPointCloudFile( filename->CStr());
        if (!inptc) {
         std::cout<<"Error";
         exit(1);
        }
       float *data;
       int datasize;
        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
        data = (float *) malloc(datasize * sizeof(float));
*/


    if (!m_tex || !m_msg)
        return 1;
    else
        return 0;
}

enum paramId
{
    k_resultF = 0, // color output
    /*k_resultR,       // float output
    k_resultG,       // float output
    k_resultB,       // float output
    k_resultA,       // float output
*/
    // Inputs
    k_filename/*,
    k_firstChannel,
    k_atlasStyle,
    k_invertT,
    k_filter,
    k_blur,
    k_lerp,
    k_missingColor,
    k_missingAlpha,
    k_linearize,
    k_mipBias,
    k_maxResolution,
    k_optimizeIndirect,
    k_colorScale,
    k_colorOffset,
    k_alphaScale,
    k_alphaOffset,
    k_saturation,
    k_manifold,
    k_manifoldQ,
    k_manifoldQradius,
    k_manifoldEnd,
    k_numParams*/
};

RixSCParamInfo const *
PxrTexture::GetParamTable()
{
    static RixSCParamInfo s_ptable[] =
    {
        // outputs
        /*RixSCParamInfo(RtUString("resultRGB"), k_RixSCColor, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultR"),  k_RixSCFloat, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultG"),  k_RixSCFloat, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultB"),  k_RixSCFloat, k_RixSCOutput),
        RixSCParamInfo(RtUString("resultA"),  k_RixSCFloat, k_RixSCOutput),*/
        RixSCParamInfo(RtUString("resultF"), k_RixSCFloat, k_RixSCOutput),
        // inputs
        RixSCParamInfo(RtUString("filename"), k_RixSCString),/*,
        RixSCParamInfo(RtUString("firstChannel"), k_RixSCInteger),
        RixSCParamInfo(RtUString("atlasStyle"), k_RixSCInteger),
        RixSCParamInfo(RtUString("invertT"), k_RixSCInteger),
        RixSCParamInfo(RtUString("filter"), k_RixSCInteger),
        RixSCParamInfo(RtUString("blur"), k_RixSCFloat),
        RixSCParamInfo(RtUString("lerp"), k_RixSCInteger),
        RixSCParamInfo(RtUString("missingColor"), k_RixSCColor),
        RixSCParamInfo(RtUString("missingAlpha"), k_RixSCFloat),
        RixSCParamInfo(RtUString("linearize"), k_RixSCInteger),
        RixSCParamInfo(RtUString("mipBias"), k_RixSCInteger),
        RixSCParamInfo(RtUString("maxResolution"), k_RixSCFloat),
        RixSCParamInfo(RtUString("optimizeIndirect"), k_RixSCInteger),

        RixSCParamInfo(RtUString("colorScale"), k_RixSCColor),
        RixSCParamInfo(RtUString("colorOffset"), k_RixSCColor),
        RixSCParamInfo(RtUString("alphaScale"), k_RixSCFloat),
        RixSCParamInfo(RtUString("alphaOffset"), k_RixSCFloat),
        RixSCParamInfo(RtUString("saturation"), k_RixSCFloat),

        RixSCParamInfo(RtUString("PxrManifold"), RtUString("manifold"), k_RixSCStructBegin),
            RixSCParamInfo(RtUString("Q"), k_RixSCPoint),
            RixSCParamInfo(RtUString("Qradius"), k_RixSCFloat),
        RixSCParamInfo(RtUString("PxrManifold"), RtUString("manifold"), k_RixSCStructEnd),
*/
            RixSCParamInfo() // end of table
        };
    return &s_ptable[0];
}

void
PxrTexture::Finalize(RixContext &ctx)
{
    PIXAR_ARGUSED(ctx);
}

int
PxrTexture::ComputeOutputParams(RixShadingContext const *sctx,
                                RtInt *noutputs, OutputSpec **outputs,
                                RtPointer instanceData,
                                RixSCParamInfo const *ignored)
{
    PIXAR_ARGUSED(instanceData);
    PIXAR_ARGUSED(ignored);

    bool indirectHit = !sctx->scTraits.primaryHit || !sctx->scTraits.eyePath ||
                       sctx->scTraits.shadingMode != k_RixSCScatterQuery;

    // Input ==================================================
    RtUString const* filename = NULL;
    sctx->EvalParam(k_filename, -1, &filename);
    if (!filename)
    {
        m_msg->Error("PxrTexture failed: filename missing");
        return 1;
    }

    RixSCType type;
    RixSCConnectionInfo cinfo;

    RixShadingContext::Allocator pool(sctx);
    OutputSpec *o = pool.AllocForPattern<OutputSpec>(3);
    *outputs = o;
    *noutputs = 1;


    RtFloat *resultF = NULL;
    resultF = pool.AllocForPattern<RtFloat>(sctx->numPts);
    o[0].paramId = k_resultF;
    o[0].detail  = k_RixSCVarying;
    o[0].value = (RtPointer) resultF;


    RtPoint3 *sP;
    //sctx->GetParamInfo(k_manifold, &type, &cinfo);
        // We want P by default (not st)
        RtPoint3 const *Q;


      //  if (*surfacePosition == k_usePo)
           // sctx->GetBuiltinVar(RixShadingContext::k_Po, &Q);
        //else
            sctx->GetBuiltinVar(RixShadingContext::k_P, &Q);

        sP = pool.AllocForPattern<RtPoint3>(sctx->numPts);
        memcpy(sP, Q, sizeof(RtPoint3)*sctx->numPts);

        // transform P in object space by default
        //
        sctx->Transform(RixShadingContext::k_AsPoints,
                        Rix::k_current, Rix::k_object, sP, NULL);



    /*
    RtInt const* ival;
    RixTexture::TxParams txParams;
    sctx->EvalParam(k_firstChannel, -1, &ival,
                    &m_firstChannel, false);
    txParams.firstchannel = *ival;

    sctx->EvalParam(k_atlasStyle, -1, &ival, &m_atlasStyle, false);
    RixTexture::TxAtlasStyle const atlasStyle = RixTexture::TxAtlasStyle(*ival);

    sctx->EvalParam(k_invertT, -1, &ival, &m_invertT, false);
    txParams.invertT = *ival;

    sctx->EvalParam(k_filter, -1, &ival, &m_filter, false);
    txParams.filter = RixTexture::TxParams::FilterType(*ival);

    bool blurVarying;
    RtFloat const *blur;
    blurVarying = (sctx->EvalParam(k_blur, -1, &blur, NULL, true) == k_RixSCVarying);
    txParams.sblurVarying = txParams.tblurVarying = blurVarying;
    txParams.sblur = txParams.tblur = blur;

    sctx->EvalParam(k_lerp, -1, &ival, &m_lerp, false);
    txParams.lerp = ((*ival) != 0);


    // 0 - off, 1 - on, 2 - auto
    sctx->EvalParam(k_linearize, -1, &ival, &m_linearize, false);
    PxrLinearizeMode linearize = PxrLinearizeMode(*ival);

    RtInt mipBias = PxrGetMipBias(sctx, k_mipBias, m_mipBias);
    RtInt maxResolution = PxrGetMaxResolution(sctx, k_maxResolution, m_maxResolution);
    RtInt const *optimizeIndirect;
    sctx->EvalParam(k_optimizeIndirect, -1, &optimizeIndirect, &m_optimizeIndirect, false);

    // opt-in for now.
    if (optimizeIndirect[0] && indirectHit)
    {
        txParams.filter = RixTexture::TxParams::FilterType::Box;
    }

    RtColorRGB const *colorScale;
    sctx->EvalParam(k_colorScale, -1, &colorScale, &m_colorScale, true);
    RtColorRGB const *colorOffset;
    sctx->EvalParam(k_colorOffset, -1, &colorOffset, &m_colorOffset, true);
    RtFloat const *alphaScale;
    sctx->EvalParam(k_alphaScale, -1, &alphaScale, &m_alphaScale, true);
    RtFloat const *alphaOffset;
    sctx->EvalParam(k_alphaOffset, -1, &alphaOffset, &m_alphaOffset, true);
    RtFloat const *saturation;
    sctx->EvalParam(k_saturation, -1, &saturation, &m_saturation, true);
*/
    // Output ==================================================
    //RixSCType type;
    //RixSCConnectionInfo cinfo;

    // Find the number of outputs
    RixSCParamInfo const* paramTable = GetParamTable();
    int numOutputs = -1;
    while (paramTable[++numOutputs].access == k_RixSCOutput) {}

   /* // Allocate and bind our outputs
    RixShadingContext::Allocator pool(sctx);
    OutputSpec* out = pool.AllocForPattern<OutputSpec>(numOutputs);
    *outputs = out;
    *noutputs = numOutputs;
*/


    float *data;
    float point[3];
    float normal[3];
    std::string input = "pointcloud_FlowShader_mask";

    int datasize;
    char* _output_file_name;
    _output_file_name = (char *) malloc((input.size()+1) * sizeof(char));
    input.copy(_output_file_name, input.size() + 1);
    _output_file_name[input.size()] = '\0';
    PtcPointCloud inptc = PtcSafeOpenPointCloudFile( filename->CStr());
        if (!inptc) {
         std::cout<<"Error";
         exit(1);
        }
        normal[0] = normal[1] = normal[2] = 0;

        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
        data = (float *) malloc(datasize * sizeof(float));

int K = 50;
float maxdist = 3;

    // looping through the different output ids
    for (unsigned i=0; i<sctx->numPts; i++)
    {

    point[0] = sP[i][0];
    point[1] = sP[i][1];
    point[2] = sP[i][2];

        int Readres = PtcGetNearestPointsData (inptc, point, normal,maxdist, K, data);
        float val = 0;
        if(Readres==1)
        {
           val= data[0];
         //  std::cout<<val<<std::endl;
            }
        /*else{
            Readres = PtcGetNearestPointsData (inptc, point, normal,1.5, K, data);
            if(Readres==1)
                val = 0.75;
            else{
                Readres = PtcGetNearestPointsData (inptc, point, normal,2, K, data);
                if(Readres==1)
                    val = 0.25;

            }
        }*/
        resultF[i] = val;
}
    return 0;
}



RIX_PATTERNCREATE
{
    PIXAR_ARGUSED(hint);

    return new PxrTexture();
}


RIX_PATTERNDESTROY
{
    delete ((PxrTexture*)pattern);
}
