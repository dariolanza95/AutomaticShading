#include "Ashader.h"

AShader::AShader(int id,float confidence = 0.0f): _id(id),_confidence(confidence) {}

AShader::AShader(): _id(0), _confidence(.0f){}

AShader::AShader(int id, BlendMode displ_blend_mode):_id(id), _confidence(.0f),_displ_blend_mode(displ_blend_mode){}

BlendMode AShader::GetDisplBlendMode(){return _displ_blend_mode;}

float AShader::GetConfidence(){return _confidence;}
void AShader::SetConfidence(float new_confidence){_confidence= new_confidence;}
int AShader::GetId(){return _id;}
