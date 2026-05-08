// (c) 2026, Cary Clark cclark2@gmail.com
// originally skia's PathOpsInverseTest.cpp,
// optimized for speed, reduced memory, and random access
#include "TinySkiaTests.h"

static void inverse(TestOptions* options) {
    const SkPathDirection dirs[] = {SkPathDirection::kCW, SkPathDirection::kCCW};
    const SkPathFillType fts[] = {
        SkPathFillType::kWinding,        SkPathFillType::kEvenOdd,
        SkPathFillType::kInverseWinding, SkPathFillType::kInverseEvenOdd
    };
    SkPath one, two;
    for (TinyOps op = TinyOps::difference; op <= TinyOps::reverseDifference; ++op) {
        if (options->skipTests(4 * 2 * 4 * 2))
            continue;
        for (auto oneFill : fts) {
            if (options->skipTests(2 * 4 * 2))
                continue;
            for (auto oneDir : dirs) {
                one.reset();
                one.setFillType(oneFill);
                one.addRect(0, 0, 6, 6, oneDir);
                if (options->skipTests(4 * 2))
                    continue;
                for (auto twoFill : fts) {
                    if (options->skipTests(2))
                        continue;
                    for (auto twoDir : dirs) {
                        if (options->skipTests(1))
                            continue;
                        two.reset();
                        two.setFillType(twoFill);
                        two.addRect(3, 3, 9, 9, twoDir);
                        options->testOne(one, two, op);
                    }
                }
            }
        }
    }
}

void V0Inverse(TestTrack* track) {
    static std::vector<TestFunc> tests = {
        TEST_FUNC_NUMBERED(inverse)
    };
    track->runTests(tests);
}
