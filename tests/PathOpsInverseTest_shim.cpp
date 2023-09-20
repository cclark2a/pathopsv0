// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsInverseTest.cpp"

void run_all_inverse_tests() {
    initializeTests(nullptr, "inverse");
    test_PathOpsInverse(nullptr);
}
