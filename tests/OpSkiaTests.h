// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#include <string>
#include "include/pathops/SkPathOps.h"
#include "include/core/SkPath.h"
#include "include/core/SkString.h"
#include "src/pathops/SkPathOpsDebug.h"
#include "src/pathops/SkPathOpsTypes.h"
#include "OpDebug.h"
#include "PathOps.h"

// since we're defining our own test harness, don't allow this to be included
#define PathOpsDebug_DEFINED
#define PathOpsExtendedTest_DEFINED
#define PathOpsTestCommon_DEFINED
#define PathOpsThreadedCommon_DEFINED

class SkBitmap;

namespace skiatest {
    struct Reporter {
        Reporter() {}
        Reporter(const char* name, const char* sub) {
            filename = name;
            subname = sub;
        }
        bool allowExtendedTest();
        bool verbose() { return false; }

        std::string filename;  // not in skia's version
        std::string subname;  // not in skia's version
    };
}

struct PathOpsDebug {
    static bool gCheckForDuplicateNames;
    static bool gJson;
};

struct PathOpsThreadState {
    PathOpsThreadState() {}
    PathOpsThreadState(int a, int b, int c, int d)
        : fA(a), fB(b), fC(c), fD(d) {
    }
    unsigned char fA;
    unsigned char fB;
    unsigned char fC;
    unsigned char fD;

    skiatest::Reporter* fReporter = nullptr;

    void outputProgress(const char* pathStr, SkPathFillType) {}
    void outputProgress(const char* pathStr, SkPathOp) {}
};

struct PathOpsThreadedTestRunner;

struct PathOpsThreadedRunnable {
    PathOpsThreadedRunnable(void (*fun)(PathOpsThreadState *),
            int a, int b, int c, int d, PathOpsThreadedTestRunner* );

    void (*fun)(PathOpsThreadState* );
};

struct DebugOneShot {
    PathOpsThreadedRunnable** append(); 
    PathOpsThreadState data;
    PathOpsThreadedRunnable* slot;
};

struct PathOpsThreadedTestRunner {
    PathOpsThreadedTestRunner(skiatest::Reporter*);
    void render();

    DebugOneShot fRunnables;
};

struct TestDesc {
    void (*fun)(skiatest::Reporter*, const char* filename);
    const char* str;
};

#define DEF_TEST(name, reporter)                                \
    static void test_##name(skiatest::Reporter*);               \
    void test_##name(skiatest::Reporter* reporter)

void RunTestSet(skiatest::Reporter* reporter, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter*, const char* filename),
        void (*skipTest)(skiatest::Reporter*, const char* filename),
        void (*stopTest)(skiatest::Reporter*, const char* filename), bool reverse);
void VerifySimplify(const SkPath& one, const SkPath& result);
void initializeTests(skiatest::Reporter* , const char* );
bool testPathOp(skiatest::Reporter*, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* filename);
bool testPathOpBase(skiatest::Reporter* , const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* filename, bool v0MayFail, bool skiaMayFail);
void testPathOpCheck(skiatest::Reporter*, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* filename, bool checkFail);
void testPathOpFuzz(skiatest::Reporter*, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* filename);
bool testPathOpFail(skiatest::Reporter* reporter, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName);
bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& , 
        const char* );
bool testSimplify(skiatest::Reporter* , const SkPath& path, const char* filename);
bool testSimplifyFail(skiatest::Reporter* , const SkPath& path, const char* filename);
void run_battle_tests();
void run_chalkboard_tests();
void run_fuzz763_tests();
void run_inverse_tests();
void run_issue3651_tests();
void run_op_circle_tests();
void run_op_rect_tests();
void run_op_tests();
void run_simplify_tests();
void run_simplify_rect_tests();
void run_tiger_tests();
void run_v0_tests();
void runTests();

inline void REPORTER_ASSERT(skiatest::Reporter* , bool test) {
    OP_ASSERT(test);
}

inline void markTestFlakyForPathKit() {
}

inline void CubicPathToQuads(const SkPath& cubicPath, SkPath* quadPath) {
    static bool oneTime = true;
    if (oneTime) {
        OpDebugOut("!!! cubic path to quads unimplemented (leaving as cubic for now)\n");
        oneTime = false;
    }
    *quadPath = cubicPath;
}

#endif
