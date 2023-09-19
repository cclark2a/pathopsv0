// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugSkiaTests.h"

#define SkString_DEFINED
#define PathOpsDebug_DEFINED
#define PathOpsExtendedTest_DEFINED
#define PathOpsThreadedCommon_DEFINED

#include "tests/PathOpsOpCircleThreadedTest.cpp"

void run_all_circle_tests() {
    test_PathOpsOpCircleThreaded(nullptr);
}
