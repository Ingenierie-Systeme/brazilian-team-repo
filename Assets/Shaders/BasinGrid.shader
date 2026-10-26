// Procedural metric grid for the basin (URP): thin lines every 1 m and thick lines every 5 m.
// The grid is computed in basin space (global matrix _BasinWorldToGrid sent by TestBasin)
// and projected on the face plane (dominant-axis triplanar), so it does not depend on the cube UVs.
Shader "TestBasin/BasinGrid"
{
    Properties
    {
        _BaseColor ("Background color", Color) = (0.80, 0.85, 0.87, 1)        // tile/concrete color
        _LineColor ("1 m line color", Color) = (0.25, 0.32, 0.38, 1)          // thin line color
        _MajorLineColor ("5 m line color", Color) = (0.06, 0.10, 0.14, 1)     // thick line color (darker)
        _LineWidth ("1 m line width [m]", Range(0.002, 0.2)) = 0.02           // thin line width in meters
        _MajorLineWidth ("5 m line width [m]", Range(0.002, 0.4)) = 0.07      // thick line width in meters
        _GridOffset ("Grid offset [m] (XYZ)", Vector) = (0, 0, 0, 0)         // manual grid offset
        _Smoothness ("Smoothness", Range(0, 1)) = 0.25                        // simple specular highlight
    }

    SubShader
    {
        // Opaque URP shader
        Tags { "RenderType" = "Opaque" "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

        HLSLINCLUDE
        // Code shared by every pass
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl" // core URP functions

        // Material properties (same block in every pass: SRP Batcher compatible)
        CBUFFER_START(UnityPerMaterial)
            half4 _BaseColor;           // background color
            half4 _LineColor;           // 1 m line color
            half4 _MajorLineColor;      // 5 m line color
            float _LineWidth;           // 1 m line width [m]
            float _MajorLineWidth;      // 5 m line width [m]
            float4 _GridOffset;         // manual offset [m]
            half _Smoothness;           // smoothness
        CBUFFER_END

        // Globals set by the TestBasin script
        float4x4 _BasinWorldToGrid;     // world → grid space matrix (basin local space)
        float _BasinGridValid;          // 1 once the matrix is sent; 0 = use world coordinates

        // Vertex input data
        struct Attributes
        {
            float4 positionOS : POSITION;   // object-space position
            float3 normalOS   : NORMAL;     // object-space normal
        };
        ENDHLSL

        // ------------------------------------------------------------------ main pass (color + lighting)
        Pass
        {
            Name "ForwardLit"                       // pass name
            Tags { "LightMode" = "UniversalForward" } // URP color pass

            HLSLPROGRAM
            #pragma vertex vert                     // vertex function
            #pragma fragment frag                   // fragment function
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE   // main light shadow variants
            #pragma multi_compile_fragment _ _SHADOWS_SOFT _SHADOWS_SOFT_LOW _SHADOWS_SOFT_MEDIUM _SHADOWS_SOFT_HIGH // soft shadows
            #pragma multi_compile_fog               // fog variants (including the underwater fog)
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl" // lights and shadows

            // Data passed from the vertex to the fragment stage
            struct Varyings
            {
                float4 positionCS : SV_POSITION;    // clip-space position
                float3 positionWS : TEXCOORD0;      // world position
                float3 normalWS   : TEXCOORD1;      // world normal
                float  fogFactor  : TEXCOORD2;      // fog factor
            };

            // Transforms the vertex and prepares the fragment data
            Varyings vert(Attributes input)
            {
                Varyings o;                                                         // output
                VertexPositionInputs pos = GetVertexPositionInputs(input.positionOS.xyz); // positions in several spaces
                o.positionCS = pos.positionCS;                                      // screen position
                o.positionWS = pos.positionWS;                                      // world position
                o.normalWS = TransformObjectToWorldNormal(input.normalOS);          // world normal
                o.fogFactor = ComputeFogFactor(pos.positionCS.z);                   // fog from depth
                return o;                                                           // return
            }

            // Intensity (0 to 1) of a family of lines along one coordinate
            float GridLine(float coord, float spacing, float width)
            {
                float d = abs(frac(coord / spacing + 0.5) - 0.5) * spacing; // distance [m] to the nearest line
                float aa = max(fwidth(coord), 1e-5);                         // size of one pixel in meters (antialiasing)
                float halfW = width * 0.5;                                   // requested half width
                float w = max(halfW, aa);                                    // never thinner than ~1 pixel (avoids vanishing/aliasing)
                float l = 1.0 - smoothstep(w - aa, w + aa, d);               // ~1 pixel soft edge
                return l * saturate(halfW / w);                              // sub-pixel lines fade out instead of flickering
            }

            // Computes the final pixel color
            half4 frag(Varyings input) : SV_Target
            {
                float3 n = normalize(input.normalWS);                                   // world normal
                bool useBasin = _BasinGridValid > 0.5;                                  // did the basin send the matrix?
                float3 g = useBasin ? mul(_BasinWorldToGrid, float4(input.positionWS, 1.0)).xyz : input.positionWS; // grid position [m]
                float3 ng = useBasin ? mul((float3x3)_BasinWorldToGrid, n) : n;         // grid-space normal
                g += _GridOffset.xyz;                                                   // apply the manual offset
                float3 an = abs(ng);                                                    // absolute normal components

                // Pick the projection plane from the dominant normal direction ("hard" triplanar)
                float2 uv = (an.y >= an.x && an.y >= an.z) ? g.xz                      // horizontal face: XZ plane
                          : (an.x >= an.z ? g.zy : g.xy);                              // ±X wall: ZY plane; ±Z wall: XY plane

                float minor = max(GridLine(uv.x, 1.0, _LineWidth), GridLine(uv.y, 1.0, _LineWidth));           // 1 m lines
                float major = max(GridLine(uv.x, 5.0, _MajorLineWidth), GridLine(uv.y, 5.0, _MajorLineWidth)); // 5 m lines

                half3 albedo = lerp(_BaseColor.rgb, _LineColor.rgb, minor);             // background + thin lines
                albedo = lerp(albedo, _MajorLineColor.rgb, major);                      // + thick lines on top

                float4 shadowCoord = TransformWorldToShadowCoord(input.positionWS);     // shadow map coordinate
                Light mainLight = GetMainLight(shadowCoord);                            // main light with shadow
                half atten = mainLight.shadowAttenuation * mainLight.distanceAttenuation; // how much light arrives
                half ndl = saturate(dot(n, mainLight.direction));                       // Lambert term
                half3 diffuse = albedo * mainLight.color * ndl * atten;                 // direct diffuse light
                half3 ambient = albedo * SampleSH(n);                                   // ambient light (sky)

                float3 v = normalize(GetWorldSpaceViewDir(input.positionWS));           // direction to the camera
                float3 h = normalize(mainLight.direction + v);                          // half vector (Blinn-Phong)
                half spec = pow(saturate(dot(n, h)), exp2(10.0 * _Smoothness + 1.0)) * _Smoothness * 0.5; // specular highlight
                half3 color = diffuse + ambient + mainLight.color * spec * atten;       // sum of the contributions

                color = MixFog(color, input.fogFactor);                                 // apply the fog
                return half4(color, 1.0);                                               // opaque color
            }
            ENDHLSL
        }

        // ------------------------------------------------------------------ shadow pass (the basin casts shadows)
        Pass
        {
            Name "ShadowCaster"                     // pass name
            Tags { "LightMode" = "ShadowCaster" }   // used when drawing the shadow map
            ZWrite On                               // write depth
            ZTest LEqual                            // default depth test
            ColorMask 0                             // no color output

            HLSLPROGRAM
            #pragma vertex vertShadow               // vertex function
            #pragma fragment fragShadow             // fragment function
            #pragma multi_compile_vertex _ _CASTING_PUNCTUAL_LIGHT_SHADOW // point or directional light
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Shadows.hlsl" // ApplyShadowBias

            float3 _LightDirection;                 // light direction (set by URP)
            float3 _LightPosition;                  // point light position (set by URP)

            // Vertex position seen from the light, with bias to avoid shadow acne
            float4 vertShadow(Attributes input) : SV_POSITION
            {
                float3 posWS = TransformObjectToWorld(input.positionOS.xyz);       // world position
                float3 nWS = TransformObjectToWorldNormal(input.normalOS);         // world normal
            #if _CASTING_PUNCTUAL_LIGHT_SHADOW
                float3 lightDir = normalize(_LightPosition - posWS);               // point light: direction to the light
            #else
                float3 lightDir = _LightDirection;                                 // directional light: fixed direction
            #endif
                float4 cs = TransformWorldToHClip(ApplyShadowBias(posWS, nWS, lightDir)); // biased position
            #if UNITY_REVERSED_Z
                cs.z = min(cs.z, UNITY_NEAR_CLIP_VALUE);                           // clamp to the near plane (reversed Z)
            #else
                cs.z = max(cs.z, UNITY_NEAR_CLIP_VALUE);                           // clamp to the near plane (regular Z)
            #endif
                return cs;                                                         // return
            }

            // Nothing to draw: only depth matters
            half4 fragShadow() : SV_Target { return 0; }
            ENDHLSL
        }

        // ------------------------------------------------------------------ depth pass (URP depth texture)
        Pass
        {
            Name "DepthOnly"                        // pass name
            Tags { "LightMode" = "DepthOnly" }      // used to build the depth texture
            ZWrite On                               // write depth
            ColorMask R                             // only the needed channel

            HLSLPROGRAM
            #pragma vertex vertDepth                // vertex function
            #pragma fragment fragDepth              // fragment function

            // Only transforms the position
            float4 vertDepth(Attributes input) : SV_POSITION
            {
                return TransformObjectToHClip(input.positionOS.xyz);               // screen position
            }

            // Nothing to draw: only depth matters
            half4 fragDepth() : SV_Target { return 0; }
            ENDHLSL
        }
    }

    FallBack "Hidden/Universal Render Pipeline/FallbackError" // if anything fails, shows the default URP pink
}
