// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyQuadralateralsThreadedTest.cpp"

void run_simplify_quadralaterals_tests() {
    skiatest::Reporter r;
    test_PathOpsSimplifyQuadralateralsThreaded(&r);
}
