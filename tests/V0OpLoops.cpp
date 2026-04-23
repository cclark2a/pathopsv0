// (c) 2026, Cary Clark cclark2@gmail.com
// originally Skia's PathOpsOpLoopThreadedTest.cpp;
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

void V0OpLoops(TestOptions* options) {
    auto test = [options](float oA, float oB, float oC, float oD) {
        for (float a = 0 ; a < 6; ++a) {
            for (float b = a + 1 ; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1 ; d < 7; ++d) {
                        if (options->skipTests(1))
                            continue;
                        // define 4 points that form two lines that often cross; one line is (a, b) (c, d)
                        OpVector v {a - c, b - d};
                        OpPoint midA { a * oA + c * (6 - oA) / 6,
                                       b * oA + d * (6 - oA) / 6 };
                        OpPoint midB { a * oB + c * (6 - oB) / 6,
                                       b * oB + d * (6 - oB) / 6 };
                        OpPoint endC { midA.x + v.dy * oC / 3,
                                       midA.y + v.dx * oC / 3 };
                        OpPoint endD { midB.x - v.dy * oD / 3,
                                       midB.y + v.dx * oD / 3 };
                        SkPath pathA, pathB;
                        pathA.moveTo(a, b);
                        pathA.cubicTo(c, d, endC.x, endC.y, endD.x, endD.y);
                        pathA.close();
                        pathB.moveTo(c, d);
                        pathB.cubicTo(endC.x, endC.y, endD.x, endD.y, a, b);
                        pathB.close();
                        if (!options->testOne(pathA, pathB, TinyOps::intersect))
                            return false;
                    }
                }
            }
        }
        return true;
    };
    const int testCount = (6 * 7 / 2) * (6 * 7 / 2);
    options->buggySkiaNumbering(testCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (int a = 0; a < 6; ++a) {  // outermost
        for (int b = a + 1; b < 7; ++b) {
            for (int c = 0 ; c < 6; ++c) {
                for (int d = c + 1; d < 7; ++d) {
                    if (!options->skipTests(testCount) && !test(a, b, c, d))
                        return;
                    options->checkTestCount(testCount);
                }
            }
            if (!options->extended) 
                return;
        }
    }
}
