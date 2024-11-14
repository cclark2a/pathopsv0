// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyTest.cpp"

void run_simplify_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplify(reporter);
}
