/*
 *  fragmentShader.glsl
 *  Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Fragment shader for Phong/Blinn-Phong Shading with one light source.

#define KERNEL_SIZE 32
#define RADIUS 0.8

varying vec3 normalEye;
varying vec4 positionEye;

varying mat4 projectionMat;
varying mat4 invProjectionMat;

uniform sampler2D depthTex, normalsTex, noiseTex;
uniform vec3 ssaoKernel[KERNEL_SIZE];
uniform bool renderDepth;
uniform float near, far;

float LinearizeDepth(vec2 uv)
{
  float z = texture2D(depthTex, uv).x;
  return (2.0 * near) / (far + near - z * (far - near));
}

vec3 blur(vec2 uv)
{
    // resolution hardcoded here, should be uniforms
    float dx = 1.0f/960; // step on x
    float dy = 1.0f/600; // step on y

    vec3 sum = vec3(0,0,0);
    for(int i = -5; i< 5; i++)
        for(int j = -5;j < 5; j++)
            sum += texture2D(normalsTex, uv + vec2(i * dx, j * dy)).rgb;
    return sum/80;
}

vec4 getViewPos(vec2 texCoord)
{
    // Calculate out of the fragment in screen space the view space position
    float x = texCoord.s * 2.0 - 1.0;
    float y = texCoord.t * 2.0 - 1.0;

    // Assume we have a normal depth range between 0.0 and 1.0
    float z = texture2D(depthTex, texCoord).r * 2.0 - 1.0;

    vec4 posProj = vec4(x, y, z, 1.0);

    vec4 posView = invProjectionMat * posProj;

    posView /= posView.w;

    return posView;
}

void main()
{
    // normalize interpolated normal, compute view vector from position
    vec3 normal = normalize( normalEye );
    vec3 view = normalize( -positionEye ).xyz;

//    // compute light vector
//    vec3 light;
//    if( gl_LightSource[0].position.w == 0.0 )
//        // directional light
//        light = normalize( gl_LightSource[0].position ).xyz;
//    else
//        // point light
//        light = normalize( gl_LightSource[0].position - positionEye ).xyz;

//    // compute the ambient component
//    //vec4 ambient = gl_FrontLightProduct[0].ambient;
//    vec4 ambient = gl_LightSource[0].ambient * gl_Color;

//    // compute the diffuse component
//    float dotLN = dot( light, normal );
//    //vec4 diffuse = gl_FrontLightProduct[0].diffuse * max( dotLN, 0.0 );
//    vec4 diffuse = gl_LightSource[0].diffuse * gl_Color * max( dotLN, 0.0 );

//    // compute the specular component
//    float factor;
//    if( dotLN > 0.0 )
//        factor = 1.0;
//    else
//        factor = 0.0;

//    // pure Phong
//    //vec3 reflect = normalize( reflect( -light, normal ) );
//    //vec4 specular =
//    //    gl_FrontLightProduct[0].specular * factor *
//    //    max( pow( dot( reflect, view ), gl_FrontMaterial.shininess ), 0.0 );

//    // modified Blinn-Phong
//    vec3 halfway = normalize( light + view );
//    vec4 specular =
//        gl_FrontLightProduct[0].specular * factor *
//        max( pow( dot( normal, halfway ), gl_FrontMaterial.shininess ), 0.0 );

    // sum the components up, defaulting alpha to 1.0
//    gl_FragColor = vec4( vec3( gl_FrontLightModelProduct.sceneColor +
//                               ambient + diffuse + specular ), 1.0 );


    if(renderDepth)
    {
        vec2 uv = gl_TexCoord[0].xy;
        vec4 posView = getViewPos(uv);

        vec2 noiseScale = vec2(240, 150);
        vec3 rvec = texture2D(noiseTex, uv * noiseScale).xyz * 2.0 - 1.0;
        vec3 tangent = normalize(rvec - normal * dot(rvec, normal));
        vec3 bitangent = cross(normal, tangent);
        mat3 tbn = mat3(tangent, bitangent, normal);

        vec3 normalValue = texture2D(normalsTex, uv).xyz * 2.0 - 1.0;
        normalValue = normalize(normalValue);

        float d = LinearizeDepth(uv);
//        if (uv.x > 0.5) // right half of the screen with non-linearized depth
//            d = texture2D(depthTex, uv).x;
        vec4 depth = vec4(vec3(d), 1.0);


        float occlusion = 0.0;
        for (int i = 0; i < KERNEL_SIZE; ++i)
        {
            // get sample position:
            vec3 samplePos = tbn * ssaoKernel[i];
            samplePos = samplePos * RADIUS + posView;

            // project sample position:
            vec4 offset = vec4(samplePos, 1.0);
            offset = projectionMat * offset;
            offset.xy /= offset.w;
            offset.xy = offset.xy * 0.5 + 0.5;

            // get sample depth:
            float z = texture2D(depthTex, offset.xy).r;
            float sampleDepth = texture2D(depthTex /*It should be linear already!*/, offset.xy).r;
//                    LinearizeDepth(offset.xy);
                // Linearizing here
//                (2.0 * near) / (far + near - z * (far - near));

            // range check & accumulate:
            float rangeCheck = abs(posView.z - sampleDepth) < RADIUS ? 1.0 : 0.0;
            occlusion += (sampleDepth <= samplePos.z ? 1.0 : 0.0) * rangeCheck;
        }
        occlusion = 1.0 - (occlusion / KERNEL_SIZE);

        gl_FragColor = vec4(normalValue, 1.0);
        gl_FragColor = vec4(rvec, 1.0);
        gl_FragColor = /*texture2D(normalsTex, uv) * */depth;
        gl_FragColor = vec4(vec3(occlusion), 1.0);
//        gl_FragColor = vec4(vec3(d), 1.0);

//        gl_FragColor = vec4( positionEye.xyz, 1);
//        gl_FragColor = vec4(blur(uv), 1) * occlusion; // with blur effect

//        This won't work, because we are rendering on a full screen quad in the main loop,
//        so all the below components are wrong. Need to have different shaders for each stage (see references)
//        gl_FragColor = vec4( vec3( gl_FrontLightModelProduct.sceneColor +
//                                   ambient /** occlusion */+ diffuse + specular ), 1.0 );
    }
    else
        // eye space per fragment normals
        gl_FragColor = vec4(normal, 1.0);

//    gl_FragColor = vec4(normal, 1.0);

//    gl_FragColor = vec4(vec3(gl_FragCoord.z/gl_FragCoord.w), 1);
}
