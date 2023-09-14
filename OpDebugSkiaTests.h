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
};

struct PathOpsThreadedTestRunner {
    PathOpsThreadedTestRunner(skiatest::Reporter*) {}
    void render() {}
};

#define DEF_TEST(name, reporter)                                \
    static void test_##name(skiatest::Reporter*);               \
    void test_##name(skiatest::Reporter* reporter)


void VerifySimplify(const SkPath& one, const SkPath& result);
void initializeTests(skiatest::Reporter* , const char* );
bool testPathOp(skiatest::Reporter*, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* filename);
bool testPathOpBase(skiatest::Reporter* , const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* filename, bool v0MayFail, bool skiaMayFail);
bool testSimplify(SkPath& path, bool useXor, SkPath& out, skiatest::PathOpsThreadState& , const char* );

#endif
