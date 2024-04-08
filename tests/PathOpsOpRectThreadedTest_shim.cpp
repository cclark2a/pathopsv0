// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsOpRectThreadedTest.cpp"

void run_op_rect_tests() {
    test_PathOpsRectsThreaded(nullptr);
}

void run_op_fast_tests() {
    test_PathOpsFastThreaded(nullptr);
}
