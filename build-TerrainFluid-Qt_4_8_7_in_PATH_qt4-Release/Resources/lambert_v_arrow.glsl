/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#version 150

// Transform matrices
/////////////////////////////////////////////////

uniform mat4 uProjMatrix;         // Projection Matrix
uniform mat4 uViewMatrix;         // View Matrix
uniform mat4 uViewMatrixNormal;   // View Matrix

// Other uniforms
/////////////////////////////////////////////////

//uniform bool uIsWater;
//uniform int uGridSize;

// Vertex Attributes
/////////////////////////////////////////////////

in vec4     point;
//in float    inTerrainHeight;
//in float    inWaterHeight;
//in float    inSediment;
//in float    inSimData;
//in vec3     inNormal;
//
//in float    inDebugXVelocity;
//in float    inDebugYVelocity;
//in float    inDebugAdvection;

// Output
/////////////////////////////////////////////////

//out vec2  vGridCoord;
//out float vTerrainHeight;
//out float vWaterHeight;
//out vec4  vFragPos;
//out vec4  vNormal;

//out vec3  vColor;
//out float vSediment;
//out float vSimData;

// Main
/////////////////////////////////////////////////

void main(void)
{
    // transform and project the vertex position
    vec4 p = point;
    vec4 vFragPos;
    // pass on some values
    //vGridCoord = inGridCoord;
    //vSediment = inSediment;
    //vSimData = inSimData;
    // surface normal
    //vec4 N = vec4(normalize(inNormal),1);
    //N = vec4(N.x,N.z,-N.y,1);
    //vNormal = uViewMatrixNormal*N ;
    
    // terrain height values
    //vTerrainHeight = inTerrainHeight+inWaterHeight;
    //vWaterHeight = inWaterHeight;

    // framgment position in camera space
    vFragPos = vec4(p.x,p.z,p.y,p.w);
    //vFragPos.xyz *= 0.01;
    vFragPos = uViewMatrix*vFragPos;
    
    gl_Position = uProjMatrix*vFragPos;


}
