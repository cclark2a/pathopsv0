// (c) 2023, Cary Clark cclark2@gmail.com
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "src/pathops/SkPathOpsPoint.h"

#include "OpSkiaTests.h"

struct CubicPts {
    static const int kPointCount = 4;
    SkDPoint fPts[kPointCount];
};

#include "tests/PathOpsOpTest.cpp"

void run_op_tests() {
    skiatest::Reporter rfail = { "opTest", "fail" };
    test_PathOpsFailOp(&rfail);
    skiatest::Reporter rmain = { "opTest", "main" };
    test_PathOpsOp(&rmain);
    skiatest::Reporter rrep = { "opTest", "rep" };
    test_PathOpsRepOp(&rrep);
}
