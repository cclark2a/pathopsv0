// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyQuadThreadedTest.cpp"

void run_simplify_quads_tests() {
    skiatest::Reporter r;
    test_PathOpsSimplifyQuadsThreaded(&r);
}
