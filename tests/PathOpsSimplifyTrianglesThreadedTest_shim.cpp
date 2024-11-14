// (c) 2023, Cary Clark cclark2@gmail.com
#include "SkiaTestShim.h"
#include "tests/PathOpsSimplifyTrianglesThreadedTest.cpp"

void run_simplify_triangles_tests(skiatest::Reporter* reporter) {
    test_PathOpsSimplifyTrianglesThreaded(reporter);
}
