// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyTest.cpp"

extern void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail);

void alt_testArc() {  // original test doesn't create an arc
    SkPath path, out;
    path.moveTo(150.000000f, 50.0021820f);
	threadableSimplifyTest(0, path, __FUNCTION__, out, false, false);
}

void run_simplify_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplify(reporter);
}
