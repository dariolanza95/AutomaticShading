#include "AShader.h"

//AShader::AShader(int id, float confidence = 0.0f, BlendMode displ_blend_mode): _id(id),_confidence(confidence) {}


AShader::AShader(int id): _id(id), _confidence(.0f),_displ_blend_mode(BlendMode::Overlay){}

AShader::AShader(int id,float confidence = 0.0f, BlendMode displ_blend_mode):_id(id), _confidence(confidence),_displ_blend_mode(displ_blend_mode){}

BlendMode AShader::GetDisplBlendMode(){return _displ_blend_mode;}

float AShader::GetConfidence(){return _confidence;}
void AShader::SetConfidence(float new_confidence){_confidence= new_confidence;}
int AShader::GetId(){return _id;}
