// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpDebugSkiaTests_DEFINED
#define OpDebugSkiaTests_DEFINED

#include <string>
#include "include/pathops/SkPathOps.h"
#include "include/core/SkPath.h"
#include "include/core/SkString.h"
#include "src/pathops/SkPathOpsDebug.h"
#include "OpDebug.h"
#include "PathOps.h"

class SkBitmap;

namespace skiatest {
    struct Reporter {
        bool allowExtendedTest() { return true; }
        bool verbose() { return false; }
    };

    struct PathOpsThreadState {
        PathOpsThreadState(int a, int b, int c, int d)
            : fA(a), fB(b), fC(c), fD(d) {
        }
        unsigned char fA;
        unsigned char fB;
        unsigned char fC;
        unsigned char fD;
        std::string fPathStr;
        const char* fKey;
        char fSerialNo[256];
        skiatest::Reporter* fReporter;
        SkBitmap* fBitmap;

        void outputProgress(const char* pathStr, SkPathFillType) {}
        void outputProgress(const char* pathStr, SkPathOp) {}
    };

}

struct PathOpsDebug {
    static bool gCheckForDuplicateNames;
    static bool gJson;
};

struct PathOpsThreadedTestRunner {
    PathOpsThreadedTestRunner(skiatest::Reporter*) {}
    void render() {}
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
bool testSimplify(SkPath& path, bool useXor, SkPath& out, skiatest::PathOpsThreadState& , 
        const char* );

void run_all_battle_tests();
void run_all_circle_tests();
void run_all_op_tests();
void run_all_simplify_rect_tests();
void run_v0_tests();

inline void REPORTER_ASSERT(skiatest::Reporter* , bool test) {
    OP_ASSERT(test);
}

inline void markTestFlakyForPathKit() {
}

inline void CubicPathToQuads(const SkPath& cubicPath, SkPath* quadPath) {
    OpDebugOut("!!! cubic path to quads unimplemented (leaving as cubic for now)\n");
    *quadPath = cubicPath;
}

#endif
