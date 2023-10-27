// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpSkiaTests.h"

#include "tests/PathOpsSimplifyTrianglesThreadedTest.cpp"

void run_simplify_triangles_tests() {
    skiatest::Reporter r;
    test_PathOpsSimplifyTrianglesThreaded(&r);
}
