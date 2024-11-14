// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsOpCircleThreadedTest.cpp"

void run_op_circle_tests(skiatest::Reporter* reporter) {
    test_PathOpsOpCircleThreaded(reporter);
}
