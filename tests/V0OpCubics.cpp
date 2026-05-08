// (c) 2025, Cary Clark cclark2@gmail.com
// originally skia's PathOpsOpCubicThreadedTest.cpp,
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

// Skia tests omitted reverse difference; put those tests at the end to preserve numbering
static void opCubics(TestOptions* options) {
    auto test = [options](float oA, float oB, float oC, float oD, bool testRevDiff) {
        for (float a = 0 ; a < 6; ++a) {
            for (float b = a + 1 ; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1 ; d < 7; ++d) {
                        for (auto e : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd }) {
                            for (auto f : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd } ) {
                                if (options->skipTests(testRevDiff ? 1 : 4))
                                    continue;
                                SkPath pathA, pathB;
                                pathA.setFillType(e);
                                pathA.moveTo(oA, oB);
                                pathA.cubicTo(oC, oD, b, a, d, c);
                                pathA.close();
                                pathB.setFillType(f);
                                pathB.moveTo(a, b);
                                pathB.cubicTo(c, d, oB, oA, oD, oC);
                                pathB.close();
                                if (testRevDiff) {
                                    if (TestDone::yes == options->testOne(pathA, pathB, TinyOps::reverseDifference))
                                            return TestDone::yes;
                                } else {
                                    for (TinyOps op = TinyOps::difference; 
                                                op <= TinyOps::exclusiveOr; ++op) {
                                        if (TestDone::yes == options->testOne(pathA, pathB, op))
                                            return TestDone::yes;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return TestDone::no;
    };
    const int oneOpCount = (6 * 7 / 2) * (6 * 7 / 2) * 2 * 2;
    const int skiaOpsCount = oneOpCount * 4;
    options->buggySkiaNumbering(skiaOpsCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (bool testRevDiff : { false, true }) {
        int testCount = testRevDiff ? oneOpCount : skiaOpsCount;
        for (float a = 0; a < 6; ++a) {  // outermost
            for (float b = a + 1; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1; d < 7; ++d) {
                        if (options->skipTests(testCount))
                            continue;
                        if (TestDone::yes == test(a, b, c, d, testRevDiff))
                            return;
                        options->checkTestCount(testCount);
                    }
                }
                if (!options->extended()) 
                    return;
            }
        }
    }
}

void V0OpCubics(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(opCubics),
    };
    track->runTests(tests);
}
