// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#include <string>

#if OP_TINY_SKIA
#include "TinySkia.h"
#define SkString_DEFINED
#define SkFloatBits_DEFINED
#define SkGeometry_DEFINED
#define SkParsePath_DEFINED
#define SkPath_DEFINED
#define SkPathOps_DEFINED
#define SkPoint_DEFINED
#define SkRandom_DEFINED
#define SkPathOpsCubic_DEFINED
#define SkPathOpsQuad_DEFINED
#define SkPathOpsPoint_DEFINED
#else
#include "include/pathops/SkPathOps.h"
#include "include/core/SkString.h"
#include "src/pathops/SkPathOpsDebug.h"
#include "src/pathops/SkPathOpsTypes.h"
#endif
#include "OpDebug.h"
#include "PathOps.h"

// since we're defining our own test harness, don't allow this to be included
#define PathOpsDebug_DEFINED
#define PathOpsExtendedTest_DEFINED
#define PathOpsTestCommon_DEFINED
#define PathOpsThreadedCommon_DEFINED
#define skiatest_Test_DEFINED

class SkBitmap;

namespace skiatest {
    struct Reporter {
        Reporter() 
            : unnamedCount(0) {
        }
        Reporter(const char* name, const char* sub)
            : unnamedCount(0) {
            filename = name;
            subname = sub;
        }
        bool allowExtendedTest();
        void bumpTestCount() {}
        bool verbose() { return false; }
        // skia's testSimplifyTrianglesMain appears to be missing the test name
        // construct one out of the name passed to initTests and a running count
        int unnamedCount;  // not in skia's version
        std::string testname;  // not in skia's version
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
    const char* fKey;

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
bool testSimplifyFuzz(skiatest::Reporter* , const SkPath& path, const char* filename);
void run_battle_tests();
void run_chalkboard_tests();
void run_fuzz763_tests();
void run_inverse_tests();
void run_issue3651_tests();
void run_op_circle_tests();
void run_op_cubic_tests();
void run_op_loop_tests();
void run_op_rect_tests();
void run_op_tests();
void run_simplify_tests();
void run_simplify_degenerate_tests();
void run_simplify_fail_tests();
void run_simplify_quadralaterals_tests();
void run_simplify_quads_tests();
void run_simplify_rect_tests();
void run_simplify_triangles_tests();
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
