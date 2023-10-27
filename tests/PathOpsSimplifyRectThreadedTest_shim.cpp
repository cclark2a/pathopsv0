// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyRectThreadedTest.cpp"

void run_simplify_rect_tests() {
    skiatest::Reporter r;
    test_PathOpsSimplifyRectsThreaded(&r);
}
