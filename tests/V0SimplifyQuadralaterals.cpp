// (c) 2026, Cary Clark cclark2@gmail.com
// copied from Skia's PathOpsSimplifyQuadralateralsThreadedTest.cpp;
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

static void simplifyQuadralaterals(TestOptions* options) {
    auto test = [options](int a, int b, int c, int d) {
        float ax = (float) (a & 0x03);
        float ay = (float) (a >> 2);
        float bx = (float) (b & 0x03);
        float by = (float) (b >> 2);
        float cx = (float) (c & 0x03);
        float cy = (float) (c >> 2);
        float dx = (float) (d & 0x03);
        float dy = (float) (d >> 2);
        for (int e = 0 ; e < 16; ++e) {
            float ex = (float) (e & 0x03);
            float ey = (float) (e >> 2);
            for (int f = e ; f < 16; ++f) {
                float fx = (float) (f & 0x03);
                float fy = (float) (f >> 2);
                for (int g = f ; g < 16; ++g) {
                    float gx = (float) (g & 0x03);
                    float gy = (float) (g >> 2);
                    for (int h = g ; h < 16; ++h) {
                        if (options->skipTests(2))
                            continue;
                        float hx = (float) (h & 0x03);
                        float hy = (float) (h >> 2);
                        SkPath path;
                        path.moveTo(ax, ay);
                        path.lineTo(bx, by);
                        path.lineTo(cx, cy);
                        path.lineTo(dx, dy);
                        path.close();
                        path.moveTo(ex, ey);
                        path.lineTo(fx, fy);
                        path.lineTo(gx, gy);
                        path.lineTo(hx, hy);
                        path.close();
                        path.setFillType(SkPathFillType::kWinding);
                        if (TestDone::yes == options->testOne(path))
                            return TestDone::yes;
                        path.setFillType(SkPathFillType::kEvenOdd);
                        if (TestDone::yes == options->testOne(path))
                            return TestDone::yes;
                    }
                }
            }
        }
        return TestDone::no;
    };
    // 4 axis triangular number (pentatope number) is 3876 for 16
    // since each path is evaluated twice (winding, even odd) skip by 7752
    const int testCount = 16 * 17 * 18 * 19 * 2 / 24;
    options->buggySkiaNumbering(testCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (int a = 0; a < 16; ++a) {
        for (int b = a ; b < 16; ++b) {
            for (int c = b ; c < 16; ++c) {
                for (int d = c; d < 16; ++d) {
                    if (!options->skipTests(testCount) && TestDone::yes == test(a, b, c, d))
                        return;
                    options->checkTestCount(testCount);
                }
                if (!options->extended()) 
                    return;
            }
        }
    }
}

void V0SimplifyQuadralaterals(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(simplifyQuadralaterals),
    };
    track->runTests(tests);
}
