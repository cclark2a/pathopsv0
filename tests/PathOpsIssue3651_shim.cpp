// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugSkiaTests.h"

#include "tests/PathOpsIssue3651.cpp"

void run_all_issue3651_tests() {
    initializeTests(nullptr, "issue3651");
    test_PathOpsIssue3651(nullptr);
}
