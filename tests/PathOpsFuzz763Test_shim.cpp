// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsFuzz763Test.cpp"

void run_fuzz763_tests(skiatest::Reporter* reporter) {
    test_PathOpsFuzz763(reporter);
}
