// (c) 2026, Cary Clark cclark2@gmail.com
// originally Skia's PathOpsSimplifyDegenerateThreadedTest.cpp;
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

static void simplifyDegenerates(TestOptions* options) {
    auto test = [options](int a, int b, int c, bool oD) {
        float ax = (float) (a & 0x03);
        float ay = (float) (a >> 2);
        float bx = (float) (b & 0x03);
        float by = (float) (b >> 2);
        float cx = (float) (c & 0x03);
        float cy = (float) (c >> 2);
        for (int d = 0; d < 16; ++d) {
            float dx = (float) (d & 0x03);
            float dy = (float) (d >> 2);
            for (int e = d ; e < 16; ++e) {
                float ex = (float) (e & 0x03);
                float ey = (float) (e >> 2);
                for (int f = d ; f < 16; ++f) {
                    float fx = (float) (f & 0x03);
                    float fy = (float) (f >> 2);
                    if (oD && (ex - dx) * (fy - dy) != (ey - dy) * (fx - dx))
                        continue;
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
                    if (TestDone::yes ==options->testOne(path))
                        return TestDone::yes;
                }
            }
        }
        return TestDone::no;
    };
    const int testCount = 2992;
    options->buggySkiaNumbering(testCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (int a = 0; a < 16; ++a) {
        int ax = a & 0x03;
        int ay = a >> 2;
        for (int b = a ; b < 16; ++b) {
            int bx = b & 0x03;
            int by = b >> 2;
            for (int c = a ; c < 16; ++c) {
                int cx = c & 0x03;
                int cy = c >> 2;
                bool abcIsATriangle = (bx - ax) * (cy - ay) != (by - ay) * (cx - ax);
                if (!options->skipTests(testCount) && TestDone::yes == test(a, b, c, abcIsATriangle))
                    return;
                options->checkTestCount(testCount);
            }
            if (!options->extended()) 
                return;
        }
    }
}

void V0SimplifyDegenerates(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(simplifyDegenerates),
    };
    track->runTests(tests);
}
