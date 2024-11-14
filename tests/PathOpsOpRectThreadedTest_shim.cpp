// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsOpRectThreadedTest.cpp"

void run_op_rect_tests(skiatest::Reporter* reporter) {
    test_PathOpsRectsThreaded(reporter);
}

void run_op_fast_tests(skiatest::Reporter* reporter) {
    test_PathOpsFastThreaded(reporter);
}
