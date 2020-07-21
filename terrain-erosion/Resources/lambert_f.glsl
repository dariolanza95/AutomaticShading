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

// Other uniforms
/////////////////////////////////////////////////

uniform bool uHardnessMode;
uniform vec4 uColor; // Material Color
uniform vec4 usedimentedTerrainColor;
// Input from Vertex Shader
/////////////////////////////////////////////////

in vec2  vGridCoord;
in float vTerrainHeight;
in float vWaterHeight;
in float vAirHeight;
in float vSediment;
in float vSedimentedTerrain;
//in float vSimData;
//in float vSimData_2;

in float vSedimentedTerrainColor;
in vec3  vP; // shading position in camera space
in vec3  vN; // shading normal in camera space
in vec4  vFragPos;
in vec3  vColor;
in vec4  vNormal;

// Output
/////////////////////////////////////////////////

out vec4 fColor;

// Main
/////////////////////////////////////////////////

void main(void)
{
    // reconstructed face normals
    //vec3 N=normalize(vec3(dFdx(gl_FragCoord.z),dFdy(gl_FragCoord.z),0.0001));
    vec3 N = normalize(vNormal.rgb);

    // toLight vectors
    vec3 L1 = normalize((uViewMatrix*vec4(-10,10,-10,1)-vFragPos).rgb);
    vec3 L2 = normalize((uViewMatrix*vec4(10,10,10,1)-vFragPos).rgb);
    float light = max(dot(N,L1),0.0f)*0.7 + max(dot(N,L2),0.0f)*0.2;

    vec4 waterColor = vec4(0,0.3,0.9,1);
//    vec4 terrainColor = vec4(0.8,0.8,0.8,1);
    vec4 terrainColor =  vec4(17.0/255.0,132.0/255.0,5.0/255.0,1);

    float factor = clamp((min(vWaterHeight,6.0)/6.0), 0.0, 1.0);
    float factor_air = clamp((min(vAirHeight,6.0)/6.0), 0.0, 1.0);
    factor = 1-pow((1-factor),4);

    factor_air = 1-pow((1-factor_air),4);

    fColor = (factor*waterColor+(1.0-factor)*terrainColor)*light;
    if(vAirHeight>0){
        vec4 fColor_air = (factor_air*waterColor+(1.0-factor_air)*terrainColor)*light;
         fColor = fColor_air;

    }
   // vec4 hardnessColor =  vec4(0,0,0,1);
   // vec4 simDataColor =  vec4(0,0,0,1);
    vec4 sedimentColor = vec4(194.0/255.0,141.0/255.0,76.0/255.0,1);
    vec4 sedimentedTerrainColor;
    if(vSedimentedTerrainColor == 0 )
        sedimentedTerrainColor = vec4(0.3,0.3,0.3,1);
    else{
        if(vSedimentedTerrainColor == 1)
            sedimentedTerrainColor = vec4(1,0,1,1);
        else{
            if(vSedimentedTerrainColor == 2)
                sedimentedTerrainColor = vec4(1,0,0,1);
            else{
                if(vSedimentedTerrainColor == 3)
                    sedimentedTerrainColor = vec4(0,1,1,1);
                else{
                    if(vSedimentedTerrainColor == 4)
                        sedimentedTerrainColor = vec4(1,1,0,1);
                    else{
                        if(vSedimentedTerrainColor == 5)
                            sedimentedTerrainColor = vec4(0,0,1,1);
                        else{
                            if(vSedimentedTerrainColor == 6)
                                sedimentedTerrainColor = vec4(0,1,0,1);
                            else{
                                if(vSedimentedTerrainColor == 7)
                                    sedimentedTerrainColor = vec4(0.7,0.3,0.7,1);
                                else{
                                    if(vSedimentedTerrainColor == 7)
                                        sedimentedTerrainColor = vec4(0.7,0.3,0.0,1);
                                    else{
                                        if(vSedimentedTerrainColor == 8)
                                            sedimentedTerrainColor = vec4(0.3,0.7,0.7,1);
                                        else
                                            sedimentedTerrainColor = vec4(0,0,0,1);
                                    }
                                }
                    }}
                    }
                }
            }
        }
    }
    float temp = 1;

    if(uHardnessMode)
    {
        //temp = vSimData;
    }
    else
        temp = 0;

    float temp2 = 1;
    float temp3 = 0;
    if(uHardnessMode)
    {
     //   temp2 = vSimData_2;
        temp3 = vSedimentedTerrain;
        temp3 = max(0,temp3);
        if(temp3>= 0)
            temp3 = 1;

    }
    else{
        temp2 = 0;
        temp3 = 0;
    }

  //  fColor = mix(fColor,vec4(0,1,0,1),vSediment);
    if(!uHardnessMode){
        fColor = mix(fColor,sedimentColor,vSediment);
    }
    else{

        fColor = mix(vec4(1,1,1,1),sedimentedTerrainColor,temp3);
    }
   // fColor = mix(fColor,hardnessColor,temp);
   // fColor = mix(fColor,simDataColor,temp2);
    float gamma = 2.2;
    fColor.rgb = pow(fColor.rgb,vec3(1,1,1)/gamma);
}
