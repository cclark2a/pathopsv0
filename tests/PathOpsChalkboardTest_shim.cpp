// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugSkiaTests.h"

#include "tests/PathOpsChalkboardTest.cpp"

void run_all_chalkboard_tests() {
    initializeTests(nullptr, "chalkboard");
    test_PathOpsChalkboard(nullptr);
}
