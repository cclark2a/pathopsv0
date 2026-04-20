// (c) 2026, Cary Clark cclark2@gmail.com
// originally skia's PathOpsInverseTest.cpp,
// optimized for speed, reduced memory, and random access
#include "TinySkia.h"
#include "TinySkiaTests.h"

void V0Inverse(TestOptions* options) {
    const SkPathDirection dirs[] = {SkPathDirection::kCW, SkPathDirection::kCCW};
    const SkPathFillType fts[] = {
        SkPathFillType::kWinding,        SkPathFillType::kEvenOdd,
        SkPathFillType::kInverseWinding, SkPathFillType::kInverseEvenOdd
    };
    SkPath one, two;
    for (TinyOps op = TinyOps::difference; op <= TinyOps::reverseDifference; ++op) {
        for (auto oneFill : fts) {
            for (auto oneDir : dirs) {
                one.reset();
                one.setFillType(oneFill);
                one.addRect(0, 0, 6, 6, oneDir);
                for (auto twoFill : fts) {
                    for (auto twoDir : dirs) {
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
