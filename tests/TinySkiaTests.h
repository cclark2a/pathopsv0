// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TinySkiaTests_DEFINED
#define TinySkiaTests_DEFINED

#include "TinySkia.h"

class SkPath;

struct TestFunc {
    void (*func)(struct TestOptions*);
    std::string name;
    bool numbered = false;
    bool mayFail = false;
};

struct TestTrack {
    void buggySkiaNumbering(int testCount) {
        indexOffset = testCount;  
        testIndex = -testCount;
        if (skip)
            skip += testCount;
    }


    bool runTests(const std::vector<TestFunc>& tests);

    bool skipTests(int count) {
        if (skip >= count) {
            skip -= count;
            testIndex += count;
            return true;
        }
        return false;
    }

    std::string baseName;  // name shared by all tests in test group
    std::string testSet;  // test to run, with trailing number removed
    int testIndex = 0;   // may be negative to adjust for bug in skia test framework
    int indexOffset = 0;
    int run = 0;
    int skip = 0;
    int toRun = 0;  // zero runs all
    bool extended = true;
};

enum class TestDone {
    no,
    yes
};

struct TestOptions {
    TestOptions(TestTrack& track, const TestFunc& func) 
    : testTrack(track)
    , testFunc(func) {
    }

    void buggySkiaNumbering(int testCount) {
        return testTrack.buggySkiaNumbering(testCount);
    }

    void checkTestCount(int testCount);

    bool extended() { 
        return testTrack.extended; 
    }

    bool skipTests(int count) {
        return testTrack.skipTests(count);
    }

    TestDone testOne(SkPath& path) {
        return testOne(path, path, TinyOps::simplify);
    }

    TestDone testOne(SkPath& left, SkPath& right, TinyOps op);

    TestTrack& testTrack;
    const TestFunc& testFunc;
    bool testCountCheck = true;  // used to verify that skip count computed by test is right
    bool ignoreRaster = false;  // set true for fuzz tests with large values
};

#define TEST_FUNC(s) { &s, #s }
#define TEST_FUNC_FAIL(s) { &s, #s, false, true }
#define TEST_FUNC_NUMBERED(s) { &s, #s, true }
#define TEST_FUNC_NUMBERED_FAIL(s) { &s, #s, true, true }

bool runTests(const std::vector<TestFunc>& , TestOptions*);

extern void V0Battles(TestTrack* );
extern void V0Chalkboard(TestTrack* );
extern void V0Fuzz763(TestTrack* );
extern void V0Inverse(TestTrack* );
extern void V0Issue3651(TestTrack* );
extern void V0Op(TestTrack* );
extern void V0OpCircles(TestTrack* );
extern void V0OpCubics(TestTrack* );
extern void V0OpFail(TestTrack* );
extern void V0OpLoops(TestTrack* );
extern void V0OpRects(TestTrack* );
extern void V0Simplify(TestTrack* );
extern void V0SimplifyDegenerates(TestTrack* );
extern void V0SimplifyFail(TestTrack* );
extern void V0SimplifyQuadralaterals(TestTrack* );
extern void V0SimplifyQuads(TestTrack* );
extern void V0SimplifyRects(TestTrack* );
extern void V0SimplifyTriangles(TestTrack* );
extern void V0Tiger(TestTrack* );
extern void V0Tests(TestTrack* );

#endif
