// (c) 2026, Cary Clark cclark2@gmail.com
// originally skia's PathOpsChalkboardTest.cpp,
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

#define B2F(x) SkBits2Float(x)

static bool chalkboard(TestOptions* options, uint64_t testlines) {
    SkPath path;
uint64_t i = 0;
path.moveTo(B2F(0x4470eed9), B2F(0x439c1ac1));  // 963.732f, 312.209f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4470dde3), B2F(0x439c63d8));  // 963.467f, 312.78f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470dbd7), B2F(0x439c3e57), B2F(0x4470c893), B2F(0x439c69fd), B2F(0x4470cfcf), B2F(0x439c297a));  // 963.435f, 312.487f, 963.134f, 312.828f, 963.247f, 312.324f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470c46b), B2F(0x439c8149), B2F(0x4470b137), B2F(0x439c2938), B2F(0x4470b5f4), B2F(0x439ca99b));  // 963.069f, 313.01f, 962.769f, 312.322f, 962.843f, 313.325f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470e842), B2F(0x439c8335), B2F(0x447125a2), B2F(0x439cce78), B2F(0x44715a2d), B2F(0x439c61ed));  // 963.629f, 313.025f, 964.588f, 313.613f, 965.409f, 312.765f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447150d5), B2F(0x439c945c));  // 965.263f, 313.159f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4471546b), B2F(0x439c87f2), B2F(0x4471579e), B2F(0x439c8085), B2F(0x44715a8f), B2F(0x439c7c4c));  // 965.319f, 313.062f, 965.369f, 313.004f, 965.415f, 312.971f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715cbc), B2F(0x439c79dd));  // 965.449f, 312.952f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715dd3), B2F(0x439c7918));  // 965.466f, 312.946f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715e56), B2F(0x439c78d6));  // 965.474f, 312.944f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715e77), B2F(0x439c78b5));  // 965.476f, 312.943f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715e77), B2F(0x439c78b5));  // 965.476f, 312.943f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715e87), B2F(0x439c78b5));  // 965.477f, 312.943f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4471a50e), B2F(0x439d05c3), B2F(0x4470fe77), B2F(0x439bb894), B2F(0x44710f9e), B2F(0x439bdb03));  // 966.579f, 314.045f, 963.976f, 311.442f, 964.244f, 311.711f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44710fae), B2F(0x439bdb24));  // 964.245f, 311.712f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44710fbe), B2F(0x439bdba7));  // 964.246f, 311.716f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44710fce), B2F(0x439be397));  // 964.247f, 311.778f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44710eb7), B2F(0x439bedf5), B2F(0x44710978), B2F(0x439bf74d), B2F(0x447105e2), B2F(0x439c0064));  // 964.23f, 311.859f, 964.148f, 311.932f, 964.092f, 312.003f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470fe86), B2F(0x439c1270), B2F(0x4470fd4f), B2F(0x439c2250), B2F(0x44712fde), B2F(0x439c33d9));  // 963.977f, 312.144f, 963.958f, 312.268f, 964.748f, 312.405f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4470fc48), B2F(0x439c3271));  // 963.942f, 312.394f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470ee13), B2F(0x439c4c2b), B2F(0x4471476b), B2F(0x439c5c0b), B2F(0x44711177), B2F(0x439c7a40));  // 963.72f, 312.595f, 965.116f, 312.719f, 964.273f, 312.955f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44712685), B2F(0x439c7648));  // 964.602f, 312.924f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x447126a6), B2F(0x439c7d31), B2F(0x44711d2d), B2F(0x439c8085), B2F(0x44711d1d), B2F(0x439c8790));  // 964.604f, 312.978f, 964.456f, 313.004f, 964.455f, 313.059f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44712675), B2F(0x439c843c));  // 964.601f, 313.033f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44713bd5), B2F(0x439c94e0), B2F(0x44713956), B2F(0x439ca065), B2F(0x44712b63), B2F(0x439cb357));  // 964.935f, 313.163f, 964.896f, 313.253f, 964.678f, 313.401f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44711af0), B2F(0x439cb5a5), B2F(0x44712459), B2F(0x439cab47), B2F(0x44711fad), B2F(0x439ca607));  // 964.421f, 313.419f, 964.568f, 313.338f, 964.495f, 313.297f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44710f1a), B2F(0x439caf3e), B2F(0x4471325d), B2F(0x439cbb26), B2F(0x4471326e), B2F(0x439cc93a));  // 964.236f, 313.369f, 964.787f, 313.462f, 964.788f, 313.572f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44712428), B2F(0x439cd501), B2F(0x44711ad0), B2F(0x439cca82), B2F(0x447113b6), B2F(0x439cc95b));  // 964.565f, 313.664f, 964.419f, 313.582f, 964.308f, 313.573f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44712b95), B2F(0x439cf20f), B2F(0x4470f550), B2F(0x439d0790), B2F(0x4471426e), B2F(0x439d21ce));  // 964.681f, 313.891f, 963.833f, 314.059f, 965.038f, 314.264f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44715072), B2F(0x439d241c), B2F(0x44715c6a), B2F(0x439d15a5), B2F(0x44716364), B2F(0x439d24c0));  // 965.257f, 314.282f, 965.444f, 314.169f, 965.553f, 314.287f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44717b22), B2F(0x439d0791), B2F(0x44715cbc), B2F(0x439cf231), B2F(0x4471475c), B2F(0x439cda20));  // 965.924f, 314.059f, 965.449f, 313.892f, 965.115f, 313.704f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4471477d), B2F(0x439ce12a));  // 965.117f, 313.759f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470fc4a), B2F(0x439cd14b), B2F(0x44715810), B2F(0x439cd0e8), B2F(0x4471372b), B2F(0x439cb272));  // 963.942f, 313.635f, 965.376f, 313.632f, 964.862f, 313.394f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x447155b2), B2F(0x439cb91a), B2F(0x44715581), B2F(0x439cc72e), B2F(0x447165f4), B2F(0x439ccbeb));  // 965.339f, 313.446f, 965.336f, 313.556f, 965.593f, 313.593f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44719e77), B2F(0x439ca2b4), B2F(0x44713979), B2F(0x439c993b), B2F(0x4471821d), B2F(0x439c7b47));  // 966.476f, 313.271f, 964.898f, 313.197f, 966.033f, 312.963f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4471847b), B2F(0x439c7dd6));  // 966.07f, 312.983f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44718b96), B2F(0x439c77b1), B2F(0x44717d81), B2F(0x439c6ebb), B2F(0x44717667), B2F(0x439c66ab));  // 966.181f, 312.935f, 965.961f, 312.865f, 965.85f, 312.802f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44716cff), B2F(0x439c6a41), B2F(0x44716842), B2F(0x439c7315), B2F(0x44716159), B2F(0x439c793a));  // 965.703f, 312.83f, 965.629f, 312.899f, 965.521f, 312.947f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44715a0d), B2F(0x439c712a), B2F(0x44713938), B2F(0x439c6f3e), B2F(0x44712b34), B2F(0x439c6d73));  // 965.407f, 312.884f, 964.894f, 312.869f, 964.675f, 312.855f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44714c19), B2F(0x439c614a), B2F(0x44711af2), B2F(0x439c61ee), B2F(0x44712b34), B2F(0x439c518c));  // 965.189f, 312.76f, 964.421f, 312.765f, 964.675f, 312.637f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x447149ab), B2F(0x439c499c), B2F(0x4471474d), B2F(0x439c5c0b), B2F(0x447157d0), B2F(0x439c6065));  // 965.151f, 312.575f, 965.114f, 312.719f, 965.372f, 312.753f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447142b1), B2F(0x439c4fa0));  // 965.042f, 312.622f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44714053), B2F(0x439c3f1d), B2F(0x44716396), B2F(0x439c3c6d), B2F(0x447173f9), B2F(0x439c3292));  // 965.005f, 312.493f, 965.556f, 312.472f, 965.812f, 312.395f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44715c7c), B2F(0x439c2628), B2F(0x44716397), B2F(0x439c3c4c), B2F(0x447142b1), B2F(0x439c3398));  // 965.445f, 312.298f, 965.556f, 312.471f, 965.042f, 312.403f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44715572), B2F(0x439c2919), B2F(0x44715bd8), B2F(0x439c10a6), B2F(0x447159bb), B2F(0x439bf68a));  // 965.335f, 312.321f, 965.435f, 312.13f, 965.402f, 311.926f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715698), B2F(0x439be2f4));  // 965.353f, 311.773f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447153f8), B2F(0x439bd95a));  // 965.312f, 311.698f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4471526f), B2F(0x439bd49e));  // 965.288f, 311.661f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4471524e), B2F(0x439bd45c));  // 965.286f, 311.659f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4471523e), B2F(0x439bd41a));  // 965.285f, 311.657f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44717148), B2F(0x439c124f), B2F(0x44715ae2), B2F(0x439be562), B2F(0x447161cb), B2F(0x439bf335));  // 965.77f, 312.143f, 965.42f, 311.792f, 965.528f, 311.9f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447161bb), B2F(0x439bf356));  // 965.527f, 311.901f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447161bb), B2F(0x439bf356));  // 965.527f, 311.901f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44716169), B2F(0x439bf3b8));  // 965.522f, 311.904f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x447160c5), B2F(0x439bf47d));  // 965.512f, 311.91f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44715f7d), B2F(0x439bf627));  // 965.492f, 311.923f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x447158f6), B2F(0x439bfeba), B2F(0x447152e1), B2F(0x439c0ac3), B2F(0x44714e15), B2F(0x439c1919));  // 965.39f, 311.99f, 965.295f, 312.084f, 965.22f, 312.196f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4471548c), B2F(0x439c10c7), B2F(0x447151bb), B2F(0x439bd7f2), B2F(0x44715927), B2F(0x439be271));  // 965.321f, 312.131f, 965.277f, 311.687f, 965.393f, 311.769f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x447156b8), B2F(0x439bd41b), B2F(0x44714c19), B2F(0x439bf356), B2F(0x44714b13), B2F(0x439c222f));  // 965.355f, 311.657f, 965.189f, 311.901f, 965.173f, 312.267f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44713dd4), B2F(0x439c4aa2), B2F(0x44712ea9), B2F(0x439c2be9), B2F(0x44712344), B2F(0x439c0085));  // 964.966f, 312.583f, 964.729f, 312.343f, 964.551f, 312.004f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x44712605), B2F(0x439c2fa0));  // 964.594f, 312.372f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x44711af3), B2F(0x439c7e9a), B2F(0x44710de4), B2F(0x439bf41b), B2F(0x4470fb65), B2F(0x439c20c7));  // 964.421f, 312.989f, 964.217f, 311.907f, 963.928f, 312.256f
if (testlines & (1LL << i++)) path.lineTo(B2F(0x4470fbb7), B2F(0x439c220f));  // 963.933f, 312.266f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470f5e4), B2F(0x439c2bc9), B2F(0x4470ef5d), B2F(0x439c9e59), B2F(0x4470e50f), B2F(0x439c85e6));  // 963.842f, 312.342f, 963.74f, 313.237f, 963.579f, 313.046f
if (testlines & (1LL << i++)) path.cubicTo(B2F(0x4470e8f6), B2F(0x439c4e35), B2F(0x4470ee98), B2F(0x439c5333), B2F(0x4470eed9), B2F(0x439c1ac1));  // 963.64f, 312.611f, 963.728f, 312.65f, 963.732f, 312.209f
SkASSERT(64 == i);
path.close();
return TestDone::yes == options->testOne(path);
}

#define ULL(x) x##ULL  // allow copy/paste of failing test from watch window
static void chalkboard_test_one(TestOptions* options) {
    chalkboard(options, ULL(1159677110206857480));
}

static void chalkboard_threaded(TestOptions* options) {
    SkRandom r;
    for (int samples = 0; samples <= 64; ++samples) {
        int testCount;
        int bitCount = samples < 32 ? samples : 64 - samples;
        int index1 = 63;
        int index2 = 62;
        switch (bitCount) {
            case 0:
                testCount = 1;
                break;
            case 1:
                testCount = 64;
                break;
            case 2:
                testCount = options->extended() ? 63 * 62 / 2 : 100;
                break;
            default:
                testCount = options->extended() ? 10000 : 100;
                break;
        }
        if (options->skipTests(testCount))
            continue;
        for (int test = 0; test < testCount; ++test) {
            uint64_t testlines;
            switch (bitCount) {
                case 0:
                    testlines = 0;
                    break;
                case 1:
                    testlines = 1LL << test;
                    break;
                case 2:
                    if (options->extended()) {
                        SkASSERT(index1 >= 1);
                        SkASSERT(index2 >= 0);
                        testlines = 1LL << index1;
                        testlines |= 1LL << index2;
                        if (--index2 < 0) {
                            --index1;
                            index2 = index1 - 1;
                        }
                        break;
                    }
                    [[fallthrough]];
                default:
                    testlines = 0;
                    for (int i = 0; i < bitCount; ++i) {
                        int bit;
                        do {
                            bit = r.nextRangeU(0, 63);
                        } while (testlines & (1LL << bit));
                        testlines |= 1LL << bit;
                    }
            }
            if (samples >= 32)
                testlines ^= 0xFFFFFFFFFFFFFFFFLL;
            if (!options->skipTests(1) && chalkboard(options, testlines))
                return;
        }
    }
}

static void chalkboard_1(TestOptions* options) {
    chalkboard(options, 0xFFFFFFFFFFFFFFFFLL);
}

void V0Chalkboard(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC(chalkboard_test_one),
        TEST_FUNC(chalkboard_1),
        TEST_FUNC_NUMBERED(chalkboard_threaded),
    };
    track->runTests(tests);
}
