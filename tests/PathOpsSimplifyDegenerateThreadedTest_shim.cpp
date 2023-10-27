// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyDegenerateThreadedTest.cpp"

void run_simplify_degenerate_tests() {
    skiatest::Reporter r;
    test_PathOpsSimplifyDegeneratesThreaded(&r);
}
