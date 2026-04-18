// (c) 2026, Cary Clark cclark2@gmail.com
// originally skia's PathOpsOpCircleThreadedTest.cpp,
// optimized for speed, reduced memory, and random access
#include "TinySkia.h"
#include "TinySkiaTests.h"

// Skia tests omitted reverse difference; put those tests at the end to preserve numbering
void V0OpCircles(TestOptions* options) {
    auto test = [options](float oA, float oB, float oC, SkPathDirection oD, bool testRevDiff) {
        SkPath pathA;
        pathA.addCircle(oA, oB, oC, oD);
        for (float a = 0 ; a < 6; ++a) {
            for (float b = a + 1 ; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (auto d : { SkPathDirection::kCW, SkPathDirection::kCCW }) {
                        for (auto e : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd }) {
                            pathA.setFillType(e);
                            for (auto f : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd } ) {
                                if (options->skipTests(testRevDiff ? 1 : 4))
                                    continue;
                                SkPath pathB;
                                pathB.setFillType(f);
                                pathB.addCircle(a, b, c, d);
                                if (testRevDiff) {
                                    if (!options->testOne(pathA, pathB, TinyOps::reverseDifference))
                                        return false;
                                } else {
                                    for (int op = 0 ; op <= (int) TinyOps::exclusiveOr; ++op)    {
                                        if (!options->testOne(pathA, pathB, (TinyOps) op))
                                            return false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return true;
    };
    const int oneOpCount = (6 * 7 / 2) * 6 * 2 * 2 * 2;
    const int skiaOpsCount = oneOpCount * 5;
    options->buggySkiaNumbering(skiaOpsCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (bool testRevDiff : { false, true }) {
        int testCount = testRevDiff ? oneOpCount : skiaOpsCount;
        for (int a = 0; a < 6; ++a) {  // outermost
            for (int b = a + 1; b < 7; ++b) {
                for (int c = 0 ; c < 6; ++c) {
                        for (auto d : { SkPathDirection::kCW, SkPathDirection::kCCW }) {
                            if (!options->skipTests(skiaOpsCount) && !test(a, b, c, d, testRevDiff))
                                return;
                        options->checkTestCount(testCount);
                    }
                }
                if (!options->extended) 
                    return;
            }
        }
    }
}
