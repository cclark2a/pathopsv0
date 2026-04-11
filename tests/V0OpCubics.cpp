// (c) 2025, Cary Clark cclark2@gmail.com
// the test below is optimized for speed, reduced memory, and random access
#include "TinySkia.h"
#include "TinySkiaTests.h"

void V0OpCubics(TestOptions* options) {
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
    const int oneOpCount = (6 * 7 / 2) * (6 * 7 / 2) * 4;
    const int skiaOpsCount = oneOpCount * 4;
    options->indexOffset = skiaOpsCount;
    options->index = -skiaOpsCount;  // skia test framework bug skips first set of tests
    options->skip += skiaOpsCount;   // ... so number those tests negative for compatibility
    // strange but true: the original skia test omitted reverse difference
    // add after the code original tests complete to preserve test number
    for (bool testRevDiff : { false, true }) {
        int testCount = testRevDiff ? oneOpCount : skiaOpsCount;
        for (float a = 0; a < 6; ++a) {  // outermost
            for (float b = a + 1; b < 7; ++b) {
                for (float c = 0 ; c < 6; ++c) {
                    for (float d = c + 1; d < 7; ++d) {
                        if (!options->skipTests(testCount) && !test(a, b, c, d, testRevDiff))
                            return;
                    }
                }
                if (!options->extended) 
                    return;
            }
        }
    }
}
