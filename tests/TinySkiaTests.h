// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TinySkiaTests_DEFINED
#define TinySkiaTests_DEFINED

#include "TinySkia.h"

class SkPath;

struct TestOptions {
    void buggySkiaNumbering(int testCount) {
        indexOffset = testCount;  
        index = -testCount;
        if (skip)
            skip += testCount;
    }

    void checkTestCount(int testCount);

    bool skipTests(int count) {
        if (skip >= count) {
            skip -= count;
            index += count;
            return true;
        }
        return false;
    }

    bool testOne(SkPath& path) {
        return testOne(path, path, TinyOps::simplify);
    }

    bool testOne(SkPath& left, SkPath& right, TinyOps op);

    std::string baseName;
    std::string customName;
    std::string testName;
    std::string testFirst;
    int index = 0;   // may be negative to adjust for bug in skia test framework
    int indexOffset = 0;
    int run = 0;
    int skip = 0;
    int toRun = 0;  // zero runs all
    bool runOne = false;
    bool extended = true;
    bool v0MayFail = false;
    bool testCountCheck = true;
    bool ignoreRaster = false;  // set true for fuzz tests with large values
};

extern void V0Battles(TestOptions* );
extern void V0Chalkboard(TestOptions* );
extern void V0Fuzz763(TestOptions* );
extern void V0Inverse(TestOptions* );
extern void V0Issue3651(TestOptions* );
extern void V0Op(TestOptions* );
extern void V0OpCircles(TestOptions* );
extern void V0OpCubics(TestOptions* );
extern void V0OpFail(TestOptions* );
extern void V0OpLoops(TestOptions* );
extern void V0OpRects(TestOptions* );
extern void V0OpFastRects(TestOptions* );
extern void V0Simplify(TestOptions* );
extern void V0SimplifyDegenerates(TestOptions* );
extern void V0SimplifyFail(TestOptions* );
extern void V0SimplifyQuadralaterals(TestOptions* );
extern void V0SimplifyQuads(TestOptions* );
extern void V0SimplifyRects(TestOptions* );
extern void V0SimplifyTriangles(TestOptions* );

#endif
