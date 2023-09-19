// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugSkiaTests.h"

#define PathOpsExtendedTest_DEFINED
#define PathOpsThreadedCommon_DEFINED

#include "tests/PathOpsSimplifyRectThreadedTest.cpp"

void run_all_simplify_rect_tests() {
    test_PathOpsSimplifyRectsThreaded(nullptr);
}
