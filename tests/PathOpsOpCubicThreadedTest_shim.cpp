// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsOpCubicThreadedTest.cpp"

void run_op_cubic_tests(skiatest::Reporter* reporter) {
    test_PathOpsOpCubicsThreaded(reporter);
}
