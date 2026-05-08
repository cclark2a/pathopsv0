// (c) 2026, Cary Clark cclark2@gmail.com
// copied from Skia's PathOpsSimplifyTrianglesThreadedTest.cpp;
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

static void simplifyTriangles(TestOptions* options) {
    auto test = [options](int a, int b, int c) {
        float ax = (float) (a & 0x03);
        float ay = (float) (a >> 2);
        float bx = (float) (b & 0x03);
        float by = (float) (b >> 2);
        float cx = (float) (c & 0x03);
        float cy = (float) (c >> 2);
        for (int d = 0; d < 15; ++d) {
            float dx = (float) (d & 0x03);
            float dy = (float) (d >> 2);
            for (int e = d + 1; e < 16; ++e) {
                float ex = (float) (e & 0x03);
                float ey = (float) (e >> 2);
                for (int f = d + 1; f < 16; ++f) {
                    if (e == f) {
                        continue;
                    }
                    float fx = (float) (f & 0x03);
                    float fy = (float) (f >> 2);
                    if ((ex - dx) * (fy - dy) == (ey - dy) * (fx - dx)) {
                        continue;
                    }
                    if (options->skipTests(2))
                        continue;
                    SkPath path;
                    path.moveTo(ax, ay);
                    path.lineTo(bx, by);
                    path.lineTo(cx, cy);
                    path.close();
                    path.moveTo(dx, dy);
                    path.lineTo(ex, ey);
                    path.lineTo(fx, fy);
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
        return TestDone::no;
    };
    const int testCount = 2064;
    options->buggySkiaNumbering(testCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (int a = 0; a < 15; ++a) {
        int ax = a & 0x03;
        int ay = a >> 2;
        for (int b = a + 1; b < 16; ++b) {
            int bx = b & 0x03;
            int by = b >> 2;
            for (int c = a + 1; c < 16; ++c) {
                if (b == c)
                    continue;
                int cx = c & 0x03;
                int cy = c >> 2;
                if ((bx - ax) * (cy - ay) == (by - ay) * (cx - ax))
                    continue;
                if (!options->skipTests(testCount) && TestDone::yes == test(a, b, c))
                    return;
                options->checkTestCount(testCount);
            }
            if (!options->extended()) 
                return;
        }
    }
}

void V0SimplifyTriangles(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(simplifyTriangles),
    };
    track->runTests(tests);
}
