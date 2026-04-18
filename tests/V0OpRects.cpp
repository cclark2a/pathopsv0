// (c) 2026, Cary Clark cclark2@gmail.com
// the test below is optimized for speed, reduced memory, and random access
#include "TinySkia.h"
#include "TinySkiaTests.h"

// four rects, of four sizes
// for 3 smaller sizes, tall, wide
    // top upper mid lower bottom aligned (3 bits, 5 values)
    // same with x (3 bits, 5 values)
// not included, square, tall, wide (2 bits)
// cw or ccw (1 bit)

// Skia tests omitted reverse difference; put those tests at the end to preserve numbering
void V0OpRects(TestOptions* options) {
    auto test = [options](float oA, float oB, float oC, float oD, bool testRevDiff) {
        SkPath pathA;
        pathA.addRect(oA, oA, oB, oB, SkPathDirection::kCW);
        pathA.addRect(oC, oC, oD, oD, SkPathDirection::kCW);
        pathA.close();
        for (float a = 0 ; a < 6; ++a) {
            for (float b = a + 1 ; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1 ; d < 7; ++d) {
                        for (auto e : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd }) {
                            pathA.setFillType(e);
                            for (auto f : { SkPathFillType::kWinding, SkPathFillType::kEvenOdd }) {
                                if (options->skipTests(testRevDiff ? 1 : 4))
                                    continue;
                                SkPath pathB;
                                pathB.setFillType(f);
                                pathB.addRect(a, a, b, b, SkPathDirection::kCW);
                                pathB.addRect(c, c, d, d, SkPathDirection::kCW);
                                pathB.close();
                                if (testRevDiff) {
                                    if (!options->testOne(pathA, pathB, TinyOps::reverseDifference))
                                        return false;
                                } else {
                                    for (int op = 0 ; op <= (int) TinyOps::exclusiveOr; ++op) {
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
                        if (!options->skipTests(testCount) && !test(a, b, c, d, testRevDiff))
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

void V0OpFastRects(TestOptions* options) {
    auto test = [options](float oA, float oB, float oC, float oD, bool testRevDiff) {
        const SkPathFillType fts[] = {
            SkPathFillType::kWinding,        SkPathFillType::kEvenOdd,
            SkPathFillType::kInverseWinding, SkPathFillType::kInverseEvenOdd
        };
        float step = options->extended ? 2 : 5;
        for (bool a : { false, true } ) {
            for (bool b : { false, true } ) {
                for (float c = 0; c < 6; c += step) {
                    for (float d = 0; d < 6; d += step) {
                        for (auto e : fts) {
                            for (auto f : fts) {
                                if (options->skipTests(testRevDiff ? 1 : 4))
                                    continue;
                                SkPath pathA, pathB;
                                pathA.setFillType(e);
                                if (a)
                                    pathA.addRect(oA, oA, oB + c, oB, SkPathDirection::kCW);
                                pathA.close();
                                pathB.setFillType(f);
                                if (b)
                                    pathB.addRect(oC, oC, oD + d, oD, SkPathDirection::kCW);
                                pathB.close();
                                if (testRevDiff) {
                                    if (!options->testOne(pathA, pathB, TinyOps::reverseDifference))
                                        return false;
                                } else {
                                    for (int op = 0; op <= (int) TinyOps::exclusiveOr; ++op) {
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
    int stepTests = options->extended ? 2 : 3;
    int oneOpCount = 2 * 2 * stepTests * stepTests * 4 * 4;
    int skiaOpsCount = oneOpCount * 4;
    options->buggySkiaNumbering(skiaOpsCount);  // skia test framework bug skips first set of tests
                                             // ... so number those tests negative for compatibility
    for (bool testRevDiff : { false, true }) {
        int testCount = testRevDiff ? oneOpCount : skiaOpsCount;
        for (float a = 0; a < 6; ++a) {  // outermost
            for (float b = a + 1; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1; d < 7; ++d) {
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
