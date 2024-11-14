// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsChalkboardTest.cpp"

void run_chalkboard_tests(skiatest::Reporter* reporter) {
    test_PathOpsChalkboard(reporter);
}
