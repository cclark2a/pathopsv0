// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyFailTest.cpp"

void run_simplify_fail_tests(skiatest::Reporter* reporter) {
	reporter->subname = "failOne";
    test_PathOpsSimplifyFailOne(reporter);
	reporter->subname = "dontFailOne";
    test_PathOpsSimplifyDontFailOne(reporter);
	reporter->subname = "fail";
    test_PathOpsSimplifyFail(reporter);
}
