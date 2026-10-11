// OSG Texture2D defaults to GL_CLAMP and LINEAR_MIPMAP_LINEAR. Filament's
// portable samplers have CLAMP_TO_EDGE, which lacks GL_CLAMP's black border.
// Restore the bilinear border coverage independently at each selected mip;
// padding the base image would change that coverage and UVs at smaller mips.
vec4 sampleLegacyClampLevel(sampler2D image, vec2 uv, int level) {
    vec2 size = vec2(textureSize(image, level));
    vec2 coverage = clamp(min(uv, vec2(1.0) - uv) * size + 0.5, 0.0, 1.0);
    return textureLod(image, uv, float(level)) * coverage.x * coverage.y;
}

vec4 sampleLegacyTexture(sampler2D image, vec2 uv) {
    vec2 size = vec2(textureSize(image, 0));
    // Compute the LOD before GL_CLAMP clamps the coordinates to [0, 1].
    float rho = max(length(dFdx(uv) * size), length(dFdy(uv) * size));
    float lastLevel = floor(log2(max(size.x, size.y)));
    float lod = clamp(log2(max(rho, 1.0)), 0.0, lastLevel);
    int first = int(floor(lod));
    int second = int(min(float(first + 1), lastLevel));
    vec2 clampedUv = clamp(uv, 0.0, 1.0);
    return mix(sampleLegacyClampLevel(image, clampedUv, first),
            sampleLegacyClampLevel(image, clampedUv, second), fract(lod));
}
