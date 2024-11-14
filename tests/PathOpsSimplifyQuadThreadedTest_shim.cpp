// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyQuadThreadedTest.cpp"

void run_simplify_quads_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplifyQuadsThreaded(reporter);
}
