// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TinySkiaTests_DEFINED
#define TinySkiaTests_DEFINED

#include "TinySkia.h"

#define THREAD_DEBUG 1
#define THREAD_DEBUG_VERBOSE 0

class SkPath;
struct TinyState;

enum class TestDone {
    no,
    yes,
    skip,
    run
};

struct TestFunc {
    void (*func)(struct TestOptions*);
    std::string name;
    bool numbered = false;
    bool mayFail = false;
};

enum class RunType {
    uninitialized,
    noDigit,
    noMatch,
    noNumber,
    notFirst,
    run,
    skip,
    smallSkip,
    test,
    zeroToRun,
};

struct TestRun {
    std::string funcName;
    int first = INT_MAX;
    int last = INT_MAX;
    RunType type = RunType::uninitialized;
};

struct TestTrack {
    void buggySkiaNumbering(int testCount) {
        indexOffset = testCount;  
    }

    TestRun* addTested(std::vector<TestRun>& , RunType );
    void addRun() {
        addTested(testRuns, RunType::test); }
    void addSkip(RunType runType) {
        addTested(testSkips, runType); }
    void addSkipRange(RunType , int count);
    bool runTests(const std::vector<TestFunc>& tests);
    void setName();
    TestDone skipInner(int count);
    bool skipTests(int count);

    std::vector<TestRun> testRuns;
    std::vector<TestRun> testSkips;
    TinyState* tinyState = nullptr;
    const TestFunc* testFunc = nullptr;
    std::string testMatch;  // if run one: test to run, with trailing number removed
    std::string testName;  // set by function currently running
    float maxError = 0;
    int runIndex = 0;   // current number to append to test func name if numbered
    int testSuffix = 0;  // number at end of named test to run
    int indexOffset = 0;  // only used for legacy tests to start with negative index
    int run = 0;  // number of tests that have run
    int skip = 0;  // number of tests to skip before running first test
    int toRun = 0;  // zero runs all
    int threadNo = -1;
    bool extended = false;
    bool hasDigits = false;
    bool runNamedTest = false;
};

struct TestOptions {
    TestOptions(TestTrack& track, const TestFunc& func) 
    : testTrack(track)
    , testFunc(func) {
    }

    void buggySkiaNumbering(int testCount) {
        testTrack.buggySkiaNumbering(testCount);
    }

    void checkTestCount(int testCount);

    bool extended() { 
        return testTrack.extended; 
    }

    bool skipTests(int count) {
        return testTrack.skipTests(count);
    }

    TestDone testLast(SkPath& left, SkPath& right, TinyOps op);

    TestDone testOne(SkPath& path) {
        return testOne(path, path, TinyOps::simplify);
    }

    TestDone testOne(SkPath& left, SkPath& right, TinyOps op);

    TestDone testPart(SkPath& left, SkPath& right, TinyOps , SkPath* result);
    TestDone testSetup(SkPath& left, SkPath& right, TinyOps , SkPath* result);

    OpDebugData debugData;
    TestTrack& testTrack;
    const TestFunc& testFunc;
    bool testCountCheck = true;  // used to verify that skip count computed by test is right
    bool ignoreRaster = false;  // set true for fuzz tests with large values
    bool firstRun = true;
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
