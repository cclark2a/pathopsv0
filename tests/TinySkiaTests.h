// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TinySkiaTests_DEFINED
#define TinySkiaTests_DEFINED

#include "OpDebug.h"

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
        ++index;
        if (--skip >= 0)
            return true;
        testName = baseName + STR(index);  // 1st test is xxx1
        (*testFunc)(path, this);
        ++run;
        return --toRun != 0;
    }

    void (*testFunc)(SkPath& , TestOptions* ) = nullptr;
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

extern void TestSimplify(SkPath& path , TestOptions* options);
extern void V0SimplifyQuads(TestOptions* options);

#endif
