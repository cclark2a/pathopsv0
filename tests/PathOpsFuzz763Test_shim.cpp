// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsFuzz763Test.cpp"

void run_all_fuzz763_tests() {
    initializeTests(nullptr, "fuzz763");
    test_PathOpsFuzz763(nullptr);
}
