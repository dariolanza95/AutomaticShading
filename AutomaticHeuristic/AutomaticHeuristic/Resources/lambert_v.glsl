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

uniform bool uIsWater;
uniform int uGridSize;

// Vertex Attributes
/////////////////////////////////////////////////

in vec3     inGridCoord;
in float    inSediment;
in float    inSimData;
in vec3     inNormal;

in float    inDebugXVelocity;
in float    inDebugYVelocity;
in float    inDebugAdvection;

// Output
/////////////////////////////////////////////////

//out vec3  vGridCoord;
out vec4  vFragPos;
out vec4  vNormal;

out vec3  vColor;
out float vSediment;
out float vSimData;

// Main
/////////////////////////////////////////////////

void main(void)
{
    // transform and project the vertex position
//    vec2 p = inGridCoord-vec2(0.5,0.5);
    vec3 p = inGridCoord;

    // pass on some values
//    vGridCoord = inGridCoord;
    vSediment = inSediment;
    vSimData = inSimData;
    // surface normal
    vec4 N = vec4(normalize(inNormal),1);
    N = vec4(N.x,N.z,-N.y,1);
    vNormal = uViewMatrixNormal*N ;

    // terrain height values

    // framgment position in camera space
    vFragPos = vec4(p.x,p.z,p.y,1);
    vFragPos = uViewMatrix*vFragPos;
    
    gl_Position = uProjMatrix*vFragPos;


}
