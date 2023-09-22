// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyTest.cpp"

void run_all_simplify_tests() {
    initializeTests(nullptr, "simplify");
    test_PathOpsSimplify(nullptr);
}
