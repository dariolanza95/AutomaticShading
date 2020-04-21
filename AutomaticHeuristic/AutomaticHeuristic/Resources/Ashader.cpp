#include "Ashader.h"

AShader::AShader(int id,float confidence = 0.0f): id(id),confidence(confidence) {}

AShader::AShader(): id(0), confidence(.0f){}

AShader::AShader(int id):id(id), confidence(.0f){}

float AShader::GetConfidence(){return confidence;}
void AShader::SetConfidence(float new_confidence){confidence= new_confidence;}
int AShader::GetId(){return id;}
