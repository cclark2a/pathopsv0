// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyDegenerateThreadedTest.cpp"

void run_simplify_degenerate_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplifyDegeneratesThreaded(reporter);
}
