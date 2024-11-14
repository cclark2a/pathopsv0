// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsTigerTest.cpp"

void run_tiger_tests(skiatest::Reporter* reporter) {
    test_PathOpsTiger(reporter);
}
