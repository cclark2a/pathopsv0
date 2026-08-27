// (c) 2026, Cary Clark cclark2@gmail.com
// originally skia's PathOpsTiger.cpp,
// optimized for speed, readability, reduced memory, and random access
#include "TinySkiaTests.h"

#define B2F(x) SkBits2Float(x)

static void tiger8(TestOptions* options) {
    SkPath path;
path.moveTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
path.quadTo(B2F(0x43f58ce4), B2F(0x435d2a04), B2F(0x43f71bd9), B2F(0x435ac7d8));  // 491.101f, 221.164f, 494.218f, 218.781f
path.quadTo(B2F(0x43f7d69d), B2F(0x4359aa35), B2F(0x43f8b3b3), B2F(0x435951c5));  // 495.677f, 217.665f, 497.404f, 217.319f
path.conicTo(B2F(0x43f8ba67), B2F(0x43594f16), B2F(0x43f8c136), B2F(0x43594dd9), B2F(0x3f7fa2b1));  // 497.456f, 217.309f, 497.509f, 217.304f, 0.998576f
path.quadTo(B2F(0x43fcc3a8), B2F(0x43589340), B2F(0x43ff01dc), B2F(0x4352e191));  // 505.529f, 216.575f, 510.015f, 210.881f
path.conicTo(B2F(0x43ff5113), B2F(0x4352187b), B2F(0x43ffb59e), B2F(0x4352b6e9), B2F(0x3f3504f3));  // 510.633f, 210.096f, 511.419f, 210.714f, 0.707107f
path.conicTo(B2F(0x43ffdc85), B2F(0x4352f435), B2F(0x43ffe4a9), B2F(0x435355e9), B2F(0x3f6ec0ae));  // 511.723f, 210.954f, 511.786f, 211.336f, 0.932628f
path.quadTo(B2F(0x4400461c), B2F(0x435b3080), B2F(0x4400b692), B2F(0x4360b229));  // 513.095f, 219.189f, 514.853f, 224.696f
path.conicTo(B2F(0x4400c662), B2F(0x43617856), B2F(0x44009920), B2F(0x4361decb), B2F(0x3f46ad5b));  // 515.1f, 225.47f, 514.393f, 225.87f, 0.776083f
path.quadTo(B2F(0x43fb4920), B2F(0x43688f50), B2F(0x43f8340f), B2F(0x4365b887));  // 502.571f, 232.56f, 496.407f, 229.721f
path.quadTo(B2F(0x43f72cd2), B2F(0x4364c612), B2F(0x43f69888), B2F(0x4362e330));  // 494.35f, 228.774f, 493.192f, 226.887f
path.quadTo(B2F(0x43f66a00), B2F(0x43624bae), B2F(0x43f64c73), B2F(0x4361ad04));  // 492.828f, 226.296f, 492.597f, 225.676f
path.quadTo(B2F(0x43f642ea), B2F(0x436179d2), B2F(0x43f63c1c), B2F(0x43614abe));  // 492.523f, 225.476f, 492.47f, 225.292f
path.quadTo(B2F(0x43f639c9), B2F(0x43613aa5), B2F(0x43f63809), B2F(0x43612cda));  // 492.451f, 225.229f, 492.438f, 225.175f
path.quadTo(B2F(0x43f63777), B2F(0x43612855), B2F(0x43f636df), B2F(0x43612357));  // 492.433f, 225.158f, 492.429f, 225.138f
path.quadTo(B2F(0x43f6368f), B2F(0x436120b2), B2F(0x43f6367b), B2F(0x43612005));  // 492.426f, 225.128f, 492.426f, 225.125f
path.lineTo(B2F(0x43f63656), B2F(0x43611ebc));  // 492.424f, 225.12f
path.lineTo(B2F(0x43f63647), B2F(0x43611e34));  // 492.424f, 225.118f
path.lineTo(B2F(0x43f6363f), B2F(0x43611df3));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f6363e), B2F(0x43611de5));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f6363f), B2F(0x43611deb));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f63647), B2F(0x43611e37));  // 492.424f, 225.118f
path.lineTo(B2F(0x43f63644), B2F(0x43611e19));  // 492.424f, 225.118f
path.quadTo(B2F(0x43f6365c), B2F(0x43611ee7), B2F(0x43f6365d), B2F(0x43611ef9));  // 492.425f, 225.121f, 492.425f, 225.121f
path.quadTo(B2F(0x43f63666), B2F(0x43611f4b), B2F(0x43f63672), B2F(0x43611fb1));  // 492.425f, 225.122f, 492.425f, 225.124f
path.quadTo(B2F(0x43f636ab), B2F(0x436121a4), B2F(0x43f636e3), B2F(0x4361236a));  // 492.427f, 225.131f, 492.429f, 225.138f
path.quadTo(B2F(0x43f636fd), B2F(0x43612443), B2F(0x43f63705), B2F(0x4361247e));  // 492.43f, 225.142f, 492.43f, 225.143f
path.quadTo(B2F(0x43f637d7), B2F(0x43612b15), B2F(0x43f638dc), B2F(0x436131b0));  // 492.436f, 225.168f, 492.444f, 225.194f
path.quadTo(B2F(0x43f63b88), B2F(0x43614303), B2F(0x43f63f62), B2F(0x43615368));  // 492.465f, 225.262f, 492.495f, 225.326f
path.quadTo(B2F(0x43f6436f), B2F(0x4361649f), B2F(0x43f648b2), B2F(0x43617468));  // 492.527f, 225.393f, 492.568f, 225.455f
path.quadTo(B2F(0x43f68760), B2F(0x43623072), B2F(0x43f6ec71), B2F(0x4361cb60));  // 493.058f, 226.189f, 493.847f, 225.794f
path.quadTo(B2F(0x43f722ef), B2F(0x436194e0), B2F(0x43f73027), B2F(0x43611df0));  // 494.273f, 225.582f, 494.376f, 225.117f
path.quadTo(B2F(0x43f73334), B2F(0x43610284), B2F(0x43f73333), B2F(0x4360e667));  // 494.4f, 225.01f, 494.4f, 224.9f
path.lineTo(B2F(0x43f63638), B2F(0x43611daf));  // 492.424f, 225.116f
path.lineTo(B2F(0x43f6b333), B2F(0x4360e666));  // 493.4f, 224.9f
path.lineTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
path.close();
path.moveTo(B2F(0x43f72ca1), B2F(0x43609572));  // 494.349f, 224.584f
path.conicTo(B2F(0x43f72ebd), B2F(0x4360a219), B2F(0x43f7302e), B2F(0x4360af1f), B2F(0x3f7fa741));  // 494.365f, 224.633f, 494.376f, 224.684f, 0.998646f
path.lineTo(B2F(0x43f63333), B2F(0x4360e667));  // 492.4f, 224.9f
path.quadTo(B2F(0x43f63333), B2F(0x4360ca4b), B2F(0x43f6363f), B2F(0x4360aede));  // 492.4f, 224.79f, 492.424f, 224.683f
path.quadTo(B2F(0x43f64377), B2F(0x436037ee), B2F(0x43f679f5), B2F(0x4360016e));  // 492.527f, 224.218f, 492.953f, 224.006f
path.quadTo(B2F(0x43f6df06), B2F(0x435f9c5c), B2F(0x43f71db4), B2F(0x43605866));  // 493.742f, 223.611f, 494.232f, 224.345f
path.quadTo(B2F(0x43f722f8), B2F(0x43606830), B2F(0x43f72704), B2F(0x43607966));  // 494.273f, 224.407f, 494.305f, 224.474f
path.quadTo(B2F(0x43f72ae0), B2F(0x436089cd), B2F(0x43f72d8a), B2F(0x43609b1e));  // 494.335f, 224.538f, 494.356f, 224.606f
path.quadTo(B2F(0x43f72e8e), B2F(0x4360a1b8), B2F(0x43f72f61), B2F(0x4360a850));  // 494.364f, 224.632f, 494.37f, 224.657f
path.quadTo(B2F(0x43f72f68), B2F(0x4360a88a), B2F(0x43f72f83), B2F(0x4360a964));  // 494.37f, 224.658f, 494.371f, 224.662f
path.quadTo(B2F(0x43f72fbb), B2F(0x4360ab2a), B2F(0x43f72ff4), B2F(0x4360ad1d));  // 494.373f, 224.669f, 494.375f, 224.676f
path.quadTo(B2F(0x43f73000), B2F(0x4360ad83), B2F(0x43f73009), B2F(0x4360add5));  // 494.375f, 224.678f, 494.375f, 224.679f
path.quadTo(B2F(0x43f7300b), B2F(0x4360ade9), B2F(0x43f73022), B2F(0x4360aeb5));  // 494.375f, 224.679f, 494.376f, 224.682f
path.lineTo(B2F(0x43f7301f), B2F(0x4360ae97));  // 494.376f, 224.682f
path.lineTo(B2F(0x43f73027), B2F(0x4360aee3));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73028), B2F(0x4360aeeb));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73027), B2F(0x4360aedf));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73021), B2F(0x4360aeaa));  // 494.376f, 224.682f
path.lineTo(B2F(0x43f73016), B2F(0x4360ae50));  // 494.376f, 224.681f
path.lineTo(B2F(0x43f73007), B2F(0x4360adc1));  // 494.375f, 224.679f
path.lineTo(B2F(0x43f72ff9), B2F(0x4360ad4d));  // 494.375f, 224.677f
path.quadTo(B2F(0x43f7300d), B2F(0x4360adf7), B2F(0x43f73031), B2F(0x4360af12));  // 494.375f, 224.68f, 494.376f, 224.684f
path.quadTo(B2F(0x43f730f0), B2F(0x4360b4f1), B2F(0x43f7320a), B2F(0x4360bc94));  // 494.382f, 224.707f, 494.391f, 224.737f
path.quadTo(B2F(0x43f73625), B2F(0x4360d8fe), B2F(0x43f73c59), B2F(0x4360fa4a));  // 494.423f, 224.848f, 494.471f, 224.978f
path.quadTo(B2F(0x43f75132), B2F(0x43616a36), B2F(0x43f772ac), B2F(0x4361d738));  // 494.634f, 225.415f, 494.896f, 225.841f
path.quadTo(B2F(0x43f7de60), B2F(0x436335ea), B2F(0x43f89f25), B2F(0x4363e779));  // 495.737f, 227.211f, 497.243f, 227.904f
path.quadTo(B2F(0x43fb3d30), B2F(0x436650a0), B2F(0x44005a14), B2F(0x43602133));  // 502.478f, 230.315f, 513.407f, 224.13f
path.lineTo(B2F(0x4400799a), B2F(0x4360ffff));  // 513.9f, 225
path.lineTo(B2F(0x44003ca2), B2F(0x43614dd5));  // 512.947f, 225.304f
path.quadTo(B2F(0x43ff92b8), B2F(0x435ba8f8), B2F(0x43fee825), B2F(0x4353aa15));  // 511.146f, 219.66f, 509.814f, 211.664f
path.lineTo(B2F(0x43ff6667), B2F(0x43537fff));  // 510.8f, 211.5f
path.lineTo(B2F(0x43ffcaf2), B2F(0x43541e6d));  // 511.586f, 212.119f
path.quadTo(B2F(0x43fd4888), B2F(0x435a7d38), B2F(0x43f8d864), B2F(0x435b4bbf));  // 506.567f, 218.489f, 497.691f, 219.296f
path.lineTo(B2F(0x43f8cccd), B2F(0x435a4ccc));  // 497.6f, 218.3f
path.lineTo(B2F(0x43f8e5e7), B2F(0x435b47d3));  // 497.796f, 219.281f
path.quadTo(B2F(0x43f84300), B2F(0x435b88fd), B2F(0x43f7b75b), B2F(0x435c5e8e));  // 496.523f, 219.535f, 495.432f, 220.369f
path.quadTo(B2F(0x43f6b984), B2F(0x435de2c4), B2F(0x43f72ca1), B2F(0x43609572));  // 493.449f, 221.886f, 494.349f, 224.584f
path.close();
    options->testOne(path);
}

// fails to include a line of edges, probably mis-sorting
static void tiger8a(TestOptions* options) {
    SkPath path;
    path.moveTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
path.quadTo(B2F(0x43f58ce4), B2F(0x435d2a04), B2F(0x43f71bd9), B2F(0x435ac7d8));  // 491.101f, 221.164f, 494.218f, 218.781f
path.quadTo(B2F(0x43f7d69d), B2F(0x4359aa35), B2F(0x43f8b3b3), B2F(0x435951c5));  // 495.677f, 217.665f, 497.404f, 217.319f
path.conicTo(B2F(0x43f8ba67), B2F(0x43594f16), B2F(0x43f8c136), B2F(0x43594dd9), B2F(0x3f7fa2b1));  // 497.456f, 217.309f, 497.509f, 217.304f, 0.998576f
path.quadTo(B2F(0x43fcc3a8), B2F(0x43589340), B2F(0x43ff01dc), B2F(0x4352e191));  // 505.529f, 216.575f, 510.015f, 210.881f
path.conicTo(B2F(0x43ff5113), B2F(0x4352187b), B2F(0x43ffb59e), B2F(0x4352b6e9), B2F(0x3f3504f3));  // 510.633f, 210.096f, 511.419f, 210.714f, 0.707107f
path.conicTo(B2F(0x43ffdc85), B2F(0x4352f435), B2F(0x43ffe4a9), B2F(0x435355e9), B2F(0x3f6ec0ae));  // 511.723f, 210.954f, 511.786f, 211.336f, 0.932628f
path.quadTo(B2F(0x4400461c), B2F(0x435b3080), B2F(0x4400b692), B2F(0x4360b229));  // 513.095f, 219.189f, 514.853f, 224.696f
path.conicTo(B2F(0x4400c662), B2F(0x43617856), B2F(0x44009920), B2F(0x4361decb), B2F(0x3f46ad5b));  // 515.1f, 225.47f, 514.393f, 225.87f, 0.776083f
path.quadTo(B2F(0x43fb4920), B2F(0x43688f50), B2F(0x43f8340f), B2F(0x4365b887));  // 502.571f, 232.56f, 496.407f, 229.721f
path.quadTo(B2F(0x43f72cd2), B2F(0x4364c612), B2F(0x43f69888), B2F(0x4362e330));  // 494.35f, 228.774f, 493.192f, 226.887f
path.quadTo(B2F(0x43f66a00), B2F(0x43624bae), B2F(0x43f64c73), B2F(0x4361ad04));  // 492.828f, 226.296f, 492.597f, 225.676f
path.quadTo(B2F(0x43f642ea), B2F(0x436179d2), B2F(0x43f63c1c), B2F(0x43614abe));  // 492.523f, 225.476f, 492.47f, 225.292f
path.quadTo(B2F(0x43f639c9), B2F(0x43613aa5), B2F(0x43f63809), B2F(0x43612cda));  // 492.451f, 225.229f, 492.438f, 225.175f
path.quadTo(B2F(0x43f63777), B2F(0x43612855), B2F(0x43f636df), B2F(0x43612357));  // 492.433f, 225.158f, 492.429f, 225.138f
path.quadTo(B2F(0x43f6368f), B2F(0x436120b2), B2F(0x43f6367b), B2F(0x43612005));  // 492.426f, 225.128f, 492.426f, 225.125f
path.lineTo(B2F(0x43f63656), B2F(0x43611ebc));  // 492.424f, 225.12f
path.lineTo(B2F(0x43f63647), B2F(0x43611e34));  // 492.424f, 225.118f
path.lineTo(B2F(0x43f6363f), B2F(0x43611df3));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f6363e), B2F(0x43611de5));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f6363f), B2F(0x43611deb));  // 492.424f, 225.117f
path.lineTo(B2F(0x43f63647), B2F(0x43611e37));  // 492.424f, 225.118f
path.lineTo(B2F(0x43f63644), B2F(0x43611e19));  // 492.424f, 225.118f
path.quadTo(B2F(0x43f6365c), B2F(0x43611ee7), B2F(0x43f6365d), B2F(0x43611ef9));  // 492.425f, 225.121f, 492.425f, 225.121f
path.quadTo(B2F(0x43f63666), B2F(0x43611f4b), B2F(0x43f63672), B2F(0x43611fb1));  // 492.425f, 225.122f, 492.425f, 225.124f
path.quadTo(B2F(0x43f636ab), B2F(0x436121a4), B2F(0x43f636e3), B2F(0x4361236a));  // 492.427f, 225.131f, 492.429f, 225.138f
path.quadTo(B2F(0x43f636fd), B2F(0x43612443), B2F(0x43f63705), B2F(0x4361247e));  // 492.43f, 225.142f, 492.43f, 225.143f
path.quadTo(B2F(0x43f637d7), B2F(0x43612b15), B2F(0x43f638dc), B2F(0x436131b0));  // 492.436f, 225.168f, 492.444f, 225.194f
path.quadTo(B2F(0x43f63b88), B2F(0x43614303), B2F(0x43f63f62), B2F(0x43615368));  // 492.465f, 225.262f, 492.495f, 225.326f
path.quadTo(B2F(0x43f6436f), B2F(0x4361649f), B2F(0x43f648b2), B2F(0x43617468));  // 492.527f, 225.393f, 492.568f, 225.455f
path.quadTo(B2F(0x43f68760), B2F(0x43623072), B2F(0x43f6ec71), B2F(0x4361cb60));  // 493.058f, 226.189f, 493.847f, 225.794f
path.quadTo(B2F(0x43f722ef), B2F(0x436194e0), B2F(0x43f73027), B2F(0x43611df0));  // 494.273f, 225.582f, 494.376f, 225.117f
path.quadTo(B2F(0x43f73334), B2F(0x43610284), B2F(0x43f73333), B2F(0x4360e667));  // 494.4f, 225.01f, 494.4f, 224.9f
path.lineTo(B2F(0x43f63638), B2F(0x43611daf));  // 492.424f, 225.116f
path.lineTo(B2F(0x43f6b333), B2F(0x4360e666));  // 493.4f, 224.9f
path.lineTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
path.close();
    options->testOne(path);
}

static TestDone tiger8a_x(TestOptions* options, uint64_t testlines) {
    SkPath path;
uint64_t i = 0;
if (testlines & (1LL << i++)) path.moveTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f58ce4), B2F(0x435d2a04), B2F(0x43f71bd9), B2F(0x435ac7d8));  // 491.101f, 221.164f, 494.218f, 218.781f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f7d69d), B2F(0x4359aa35), B2F(0x43f8b3b3), B2F(0x435951c5));  // 495.677f, 217.665f, 497.404f, 217.319f
if (testlines & (1LL << i++)) path.conicTo(B2F(0x43f8ba67), B2F(0x43594f16), B2F(0x43f8c136), B2F(0x43594dd9), B2F(0x3f7fa2b1));  // 497.456f, 217.309f, 497.509f, 217.304f, 0.998576f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43fcc3a8), B2F(0x43589340), B2F(0x43ff01dc), B2F(0x4352e191));  // 505.529f, 216.575f, 510.015f, 210.881f
if (testlines & (1LL << i++)) path.conicTo(B2F(0x43ff5113), B2F(0x4352187b), B2F(0x43ffb59e), B2F(0x4352b6e9), B2F(0x3f3504f3));  // 510.633f, 210.096f, 511.419f, 210.714f, 0.707107f
if (testlines & (1LL << i++)) path.conicTo(B2F(0x43ffdc85), B2F(0x4352f435), B2F(0x43ffe4a9), B2F(0x435355e9), B2F(0x3f6ec0ae));  // 511.723f, 210.954f, 511.786f, 211.336f, 0.932628f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x4400461c), B2F(0x435b3080), B2F(0x4400b692), B2F(0x4360b229));  // 513.095f, 219.189f, 514.853f, 224.696f
if (testlines & (1LL << i++)) path.conicTo(B2F(0x4400c662), B2F(0x43617856), B2F(0x44009920), B2F(0x4361decb), B2F(0x3f46ad5b));  // 515.1f, 225.47f, 514.393f, 225.87f, 0.776083f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43fb4920), B2F(0x43688f50), B2F(0x43f8340f), B2F(0x4365b887));  // 502.571f, 232.56f, 496.407f, 229.721f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f72cd2), B2F(0x4364c612), B2F(0x43f69888), B2F(0x4362e330));  // 494.35f, 228.774f, 493.192f, 226.887f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f66a00), B2F(0x43624bae), B2F(0x43f64c73), B2F(0x4361ad04));  // 492.828f, 226.296f, 492.597f, 225.676f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f642ea), B2F(0x436179d2), B2F(0x43f63c1c), B2F(0x43614abe));  // 492.523f, 225.476f, 492.47f, 225.292f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f639c9), B2F(0x43613aa5), B2F(0x43f63809), B2F(0x43612cda));  // 492.451f, 225.229f, 492.438f, 225.175f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f63777), B2F(0x43612855), B2F(0x43f636df), B2F(0x43612357));  // 492.433f, 225.158f, 492.429f, 225.138f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f6368f), B2F(0x436120b2), B2F(0x43f6367b), B2F(0x43612005));  // 492.426f, 225.128f, 492.426f, 225.125f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63656), B2F(0x43611ebc));  // 492.424f, 225.12f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63647), B2F(0x43611e34));  // 492.424f, 225.118f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f6363f), B2F(0x43611df3));  // 492.424f, 225.117f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f6363e), B2F(0x43611de5));  // 492.424f, 225.117f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f6363f), B2F(0x43611deb));  // 492.424f, 225.117f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63647), B2F(0x43611e37));  // 492.424f, 225.118f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63644), B2F(0x43611e19));  // 492.424f, 225.118f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f6365c), B2F(0x43611ee7), B2F(0x43f6365d), B2F(0x43611ef9));  // 492.425f, 225.121f, 492.425f, 225.121f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f63666), B2F(0x43611f4b), B2F(0x43f63672), B2F(0x43611fb1));  // 492.425f, 225.122f, 492.425f, 225.124f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f636ab), B2F(0x436121a4), B2F(0x43f636e3), B2F(0x4361236a));  // 492.427f, 225.131f, 492.429f, 225.138f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f636fd), B2F(0x43612443), B2F(0x43f63705), B2F(0x4361247e));  // 492.43f, 225.142f, 492.43f, 225.143f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f637d7), B2F(0x43612b15), B2F(0x43f638dc), B2F(0x436131b0));  // 492.436f, 225.168f, 492.444f, 225.194f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f63b88), B2F(0x43614303), B2F(0x43f63f62), B2F(0x43615368));  // 492.465f, 225.262f, 492.495f, 225.326f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f6436f), B2F(0x4361649f), B2F(0x43f648b2), B2F(0x43617468));  // 492.527f, 225.393f, 492.568f, 225.455f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f68760), B2F(0x43623072), B2F(0x43f6ec71), B2F(0x4361cb60));  // 493.058f, 226.189f, 493.847f, 225.794f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f722ef), B2F(0x436194e0), B2F(0x43f73027), B2F(0x43611df0));  // 494.273f, 225.582f, 494.376f, 225.117f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f73334), B2F(0x43610284), B2F(0x43f73333), B2F(0x4360e667));  // 494.4f, 225.01f, 494.4f, 224.9f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63638), B2F(0x43611daf));  // 492.424f, 225.116f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f6b333), B2F(0x4360e666));  // 493.4f, 224.9f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f639c5), B2F(0x4361375a));  // 492.451f, 225.216f
if (testlines & (1LL << i++)) path.close();
    return options->testOne(path);
}

#define ULL(x) x##ULL

static void tiger8a_h_1(TestOptions* options) {
    uint64_t testlines = ULL(274035693768);
    tiger8a_x(options, testlines);
}

static TestDone tiger8b_x(TestOptions* options, uint64_t testlines) {
    SkPath path;
uint64_t i = 0;
if (testlines & (1LL << i++)) path.moveTo(B2F(0x43f72ca1), B2F(0x43609572));  // 494.349f, 224.584f
if (testlines & (1LL << i++)) path.conicTo(B2F(0x43f72ebd), B2F(0x4360a219), B2F(0x43f7302e), B2F(0x4360af1f), B2F(0x3f7fa741));  // 494.365f, 224.633f, 494.376f, 224.684f, 0.998646f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f63333), B2F(0x4360e667));  // 492.4f, 224.9f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f63333), B2F(0x4360ca4b), B2F(0x43f6363f), B2F(0x4360aede));  // 492.4f, 224.79f, 492.424f, 224.683f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f64377), B2F(0x436037ee), B2F(0x43f679f5), B2F(0x4360016e));  // 492.527f, 224.218f, 492.953f, 224.006f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f6df06), B2F(0x435f9c5c), B2F(0x43f71db4), B2F(0x43605866));  // 493.742f, 223.611f, 494.232f, 224.345f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f722f8), B2F(0x43606830), B2F(0x43f72704), B2F(0x43607966));  // 494.273f, 224.407f, 494.305f, 224.474f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f72ae0), B2F(0x436089cd), B2F(0x43f72d8a), B2F(0x43609b1e));  // 494.335f, 224.538f, 494.356f, 224.606f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f72e8e), B2F(0x4360a1b8), B2F(0x43f72f61), B2F(0x4360a850));  // 494.364f, 224.632f, 494.37f, 224.657f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f72f68), B2F(0x4360a88a), B2F(0x43f72f83), B2F(0x4360a964));  // 494.37f, 224.658f, 494.371f, 224.662f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f72fbb), B2F(0x4360ab2a), B2F(0x43f72ff4), B2F(0x4360ad1d));  // 494.373f, 224.669f, 494.375f, 224.676f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f73000), B2F(0x4360ad83), B2F(0x43f73009), B2F(0x4360add5));  // 494.375f, 224.678f, 494.375f, 224.679f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f7300b), B2F(0x4360ade9), B2F(0x43f73022), B2F(0x4360aeb5));  // 494.375f, 224.679f, 494.376f, 224.682f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f7301f), B2F(0x4360ae97));  // 494.376f, 224.682f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73027), B2F(0x4360aee3));  // 494.376f, 224.683f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73028), B2F(0x4360aeeb));  // 494.376f, 224.683f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73027), B2F(0x4360aedf));  // 494.376f, 224.683f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73021), B2F(0x4360aeaa));  // 494.376f, 224.682f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73016), B2F(0x4360ae50));  // 494.376f, 224.681f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f73007), B2F(0x4360adc1));  // 494.375f, 224.679f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f72ff9), B2F(0x4360ad4d));  // 494.375f, 224.677f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f7300d), B2F(0x4360adf7), B2F(0x43f73031), B2F(0x4360af12));  // 494.375f, 224.68f, 494.376f, 224.684f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f730f0), B2F(0x4360b4f1), B2F(0x43f7320a), B2F(0x4360bc94));  // 494.382f, 224.707f, 494.391f, 224.737f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f73625), B2F(0x4360d8fe), B2F(0x43f73c59), B2F(0x4360fa4a));  // 494.423f, 224.848f, 494.471f, 224.978f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f75132), B2F(0x43616a36), B2F(0x43f772ac), B2F(0x4361d738));  // 494.634f, 225.415f, 494.896f, 225.841f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f7de60), B2F(0x436335ea), B2F(0x43f89f25), B2F(0x4363e779));  // 495.737f, 227.211f, 497.243f, 227.904f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43fb3d30), B2F(0x436650a0), B2F(0x44005a14), B2F(0x43602133));  // 502.478f, 230.315f, 513.407f, 224.13f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4400799a), B2F(0x4360ffff));  // 513.9f, 225
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44003ca2), B2F(0x43614dd5));  // 512.947f, 225.304f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43ff92b8), B2F(0x435ba8f8), B2F(0x43fee825), B2F(0x4353aa15));  // 511.146f, 219.66f, 509.814f, 211.664f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43ff6667), B2F(0x43537fff));  // 510.8f, 211.5f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43ffcaf2), B2F(0x43541e6d));  // 511.586f, 212.119f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43fd4888), B2F(0x435a7d38), B2F(0x43f8d864), B2F(0x435b4bbf));  // 506.567f, 218.489f, 497.691f, 219.296f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f8cccd), B2F(0x435a4ccc));  // 497.6f, 218.3f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x43f8e5e7), B2F(0x435b47d3));  // 497.796f, 219.281f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f84300), B2F(0x435b88fd), B2F(0x43f7b75b), B2F(0x435c5e8e));  // 496.523f, 219.535f, 495.432f, 220.369f
if (testlines & (1LL << i++)) path.quadTo(B2F(0x43f6b984), B2F(0x435de2c4), B2F(0x43f72ca1), B2F(0x43609572));  // 493.449f, 221.886f, 494.349f, 224.584f
if (testlines & (1LL << i++)) path.close();
    return options->testOne(path);
}

static void tiger8b_h_1(TestOptions* options) {
    uint64_t testlines = ULL(171655792639);
    tiger8b_x(options, testlines);
}

static void tiger_threaded(TestOptions* options) {
    for (int ab = 0; ab < 2; ++ab) {
        SkRandom r;
        int testCount = options->extended() ? 10000 : 100;
        for (int samples = 2; samples < 37; ++samples) {
            for (int tests = 1; tests <= testCount; ++tests) {
                uint64_t testlines = 0;
                for (int i = 0; i < samples; ++i) {
                    int bit;
                    do {
                        bit = r.nextRangeU(0, 38);
                    } while (testlines & (1LL << bit));
                    testlines |= 1LL << bit;
                }
                if (options->skipTests(1))
                    continue;
                if (TestDone::yes == (ab ? tiger8b_x(options, testlines) 
                        : tiger8a_x(options, testlines)))
                    return;
            }
        }
    }
}

// tries to add same edge twice
static void tiger8b(TestOptions* options) {
    SkPath path;
path.moveTo(B2F(0x43f72ca1), B2F(0x43609572));  // 494.349f, 224.584f
path.conicTo(B2F(0x43f72ebd), B2F(0x4360a219), B2F(0x43f7302e), B2F(0x4360af1f), B2F(0x3f7fa741));  // 494.365f, 224.633f, 494.376f, 224.684f, 0.998646f
path.lineTo(B2F(0x43f63333), B2F(0x4360e667));  // 492.4f, 224.9f
path.quadTo(B2F(0x43f63333), B2F(0x4360ca4b), B2F(0x43f6363f), B2F(0x4360aede));  // 492.4f, 224.79f, 492.424f, 224.683f
path.quadTo(B2F(0x43f64377), B2F(0x436037ee), B2F(0x43f679f5), B2F(0x4360016e));  // 492.527f, 224.218f, 492.953f, 224.006f
path.quadTo(B2F(0x43f6df06), B2F(0x435f9c5c), B2F(0x43f71db4), B2F(0x43605866));  // 493.742f, 223.611f, 494.232f, 224.345f
path.quadTo(B2F(0x43f722f8), B2F(0x43606830), B2F(0x43f72704), B2F(0x43607966));  // 494.273f, 224.407f, 494.305f, 224.474f
path.quadTo(B2F(0x43f72ae0), B2F(0x436089cd), B2F(0x43f72d8a), B2F(0x43609b1e));  // 494.335f, 224.538f, 494.356f, 224.606f
path.quadTo(B2F(0x43f72e8e), B2F(0x4360a1b8), B2F(0x43f72f61), B2F(0x4360a850));  // 494.364f, 224.632f, 494.37f, 224.657f
path.quadTo(B2F(0x43f72f68), B2F(0x4360a88a), B2F(0x43f72f83), B2F(0x4360a964));  // 494.37f, 224.658f, 494.371f, 224.662f
path.quadTo(B2F(0x43f72fbb), B2F(0x4360ab2a), B2F(0x43f72ff4), B2F(0x4360ad1d));  // 494.373f, 224.669f, 494.375f, 224.676f
path.quadTo(B2F(0x43f73000), B2F(0x4360ad83), B2F(0x43f73009), B2F(0x4360add5));  // 494.375f, 224.678f, 494.375f, 224.679f
path.quadTo(B2F(0x43f7300b), B2F(0x4360ade9), B2F(0x43f73022), B2F(0x4360aeb5));  // 494.375f, 224.679f, 494.376f, 224.682f
path.lineTo(B2F(0x43f7301f), B2F(0x4360ae97));  // 494.376f, 224.682f
path.lineTo(B2F(0x43f73027), B2F(0x4360aee3));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73028), B2F(0x4360aeeb));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73027), B2F(0x4360aedf));  // 494.376f, 224.683f
path.lineTo(B2F(0x43f73021), B2F(0x4360aeaa));  // 494.376f, 224.682f
path.lineTo(B2F(0x43f73016), B2F(0x4360ae50));  // 494.376f, 224.681f
path.lineTo(B2F(0x43f73007), B2F(0x4360adc1));  // 494.375f, 224.679f
path.lineTo(B2F(0x43f72ff9), B2F(0x4360ad4d));  // 494.375f, 224.677f
path.quadTo(B2F(0x43f7300d), B2F(0x4360adf7), B2F(0x43f73031), B2F(0x4360af12));  // 494.375f, 224.68f, 494.376f, 224.684f
path.quadTo(B2F(0x43f730f0), B2F(0x4360b4f1), B2F(0x43f7320a), B2F(0x4360bc94));  // 494.382f, 224.707f, 494.391f, 224.737f
path.quadTo(B2F(0x43f73625), B2F(0x4360d8fe), B2F(0x43f73c59), B2F(0x4360fa4a));  // 494.423f, 224.848f, 494.471f, 224.978f
path.quadTo(B2F(0x43f75132), B2F(0x43616a36), B2F(0x43f772ac), B2F(0x4361d738));  // 494.634f, 225.415f, 494.896f, 225.841f
path.quadTo(B2F(0x43f7de60), B2F(0x436335ea), B2F(0x43f89f25), B2F(0x4363e779));  // 495.737f, 227.211f, 497.243f, 227.904f
path.quadTo(B2F(0x43fb3d30), B2F(0x436650a0), B2F(0x44005a14), B2F(0x43602133));  // 502.478f, 230.315f, 513.407f, 224.13f
path.lineTo(B2F(0x4400799a), B2F(0x4360ffff));  // 513.9f, 225
path.lineTo(B2F(0x44003ca2), B2F(0x43614dd5));  // 512.947f, 225.304f
path.quadTo(B2F(0x43ff92b8), B2F(0x435ba8f8), B2F(0x43fee825), B2F(0x4353aa15));  // 511.146f, 219.66f, 509.814f, 211.664f
path.lineTo(B2F(0x43ff6667), B2F(0x43537fff));  // 510.8f, 211.5f
path.lineTo(B2F(0x43ffcaf2), B2F(0x43541e6d));  // 511.586f, 212.119f
path.quadTo(B2F(0x43fd4888), B2F(0x435a7d38), B2F(0x43f8d864), B2F(0x435b4bbf));  // 506.567f, 218.489f, 497.691f, 219.296f
path.lineTo(B2F(0x43f8cccd), B2F(0x435a4ccc));  // 497.6f, 218.3f
path.lineTo(B2F(0x43f8e5e7), B2F(0x435b47d3));  // 497.796f, 219.281f
path.quadTo(B2F(0x43f84300), B2F(0x435b88fd), B2F(0x43f7b75b), B2F(0x435c5e8e));  // 496.523f, 219.535f, 495.432f, 220.369f
path.quadTo(B2F(0x43f6b984), B2F(0x435de2c4), B2F(0x43f72ca1), B2F(0x43609572));  // 493.449f, 221.886f, 494.349f, 224.584f
path.close();
    options->testOne(path);
}

void V0Tiger(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC(tiger8a_h_1),
        TEST_FUNC(tiger8a),
        TEST_FUNC(tiger8b_h_1),
        TEST_FUNC(tiger8b),
        TEST_FUNC(tiger8),
        TEST_FUNC_NUMBERED(tiger_threaded),
    };
    track->runTests(tests);
}
