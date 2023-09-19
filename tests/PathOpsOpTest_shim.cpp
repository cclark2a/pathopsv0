// (c) 2023, Cary Clark cclark2@gmail.com
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "src/pathops/SkPathOpsPoint.h"

#include "OpDebugSkiaTests.h"

struct CubicPts {
    static const int kPointCount = 4;
    SkDPoint fPts[kPointCount];
};

#define PathOpsDebug_DEFINED
#define PathOpsExtendedTest_DEFINED
#define PathOpsTestCommon_DEFINED

#include "tests/PathOpsOpTest.cpp"

void run_all_op_tests() {
    test_PathOpsFailOp(nullptr);
    test_PathOpsOp(nullptr);
    test_PathOpsRepOp(nullptr);
}
