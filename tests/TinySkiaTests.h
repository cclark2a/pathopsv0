// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TinySkiaTests_DEFINED
#define TinySkiaTests_DEFINED

#include "TinySkia.h"

class SkPath;

struct TestOptions {
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

    bool testOne(SkPath& left, SkPath& right, TinyOps op) {
        ++index;
        if (--skip >= 0)
            return true;
        testName = baseName + STR(index);
        testOp(left, right, op);
        ++run;
        return --toRun != 0;
    }

    void testOp(SkPath& left, SkPath& right, TinyOps op);

    std::string baseName;
    std::string testName;
    std::string testFirst;
    int index = 0;   // may be negative to adjust for bug in skia test framework
    int indexOffset = 0;
    int run = 0;
    int skip = 0;
    int toRun = 0;  // zero runs all
    bool extended = true;
    bool v0MayFail = false;
};

// extern void TestSimplify(SkPath& path, TestOptions* options);
extern void V0SimplifyQuads(TestOptions* options);
extern void V0OpCubics(TestOptions* options);

#endif
