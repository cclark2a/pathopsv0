// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyFailTest.cpp"

void run_simplify_fail_tests() {
    test_PathOpsSimplifyFailOne(nullptr);
    test_PathOpsSimplifyDontFailOne(nullptr);
    test_PathOpsSimplifyFail(nullptr);
}
