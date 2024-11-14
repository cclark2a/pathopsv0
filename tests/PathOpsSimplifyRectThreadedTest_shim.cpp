// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyRectThreadedTest.cpp"

void run_simplify_rect_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplifyRectsThreaded(reporter);
}
