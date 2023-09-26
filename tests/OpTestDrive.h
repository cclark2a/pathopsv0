// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h"
#endif

#define OP_SHOW_TEST_NAME 0

// issue3517:  long and skinny; don't know what's going on
//             unterminated ends of contour are edges 1434, 

// thread_circles7489:  pair of conics do not find intersection
//                      when reduced to line/conic, there's error finding roots

// grshapearcs1:  very complicated; defer. Asserts in OpWinder::AddLineCurveIntersection
//                a 'more code needed?' alert that oSegment pinned to bounds

// fuzzhang_1: succeeds in skia, fails in v0

// testQuadratic75: (simplify) needs lookahead in matchLinks to find best disabled edge
// testQuadratic67x: links unsortable to edge crossing boundary; simple fix breaks loops44i

// battleOp33: fails to find intersection matching edge (have not debugged)

// tiger8b: no matching link found (have not debugged)

// fast802: fails verify (have not debugged)

#define TEST_PATH_OP_EXCEPTIONS "issue3517", "thread_circles7489"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "testQuadratic75", "testQuadratic67x"
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc"
#define TEST_PATH_OP_MAP_TO_FUZZ  "fuzzhang_1"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST "issue3651_7", "thread_circles7490", \
        "fuzz763_2674194", "battleOp33", "tiger8b", "fast802"

#define TEST_PATH_OP_FIRST "" /* e.g., "tiger8b_x2" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "rect" /* e.g., "tiger" to run this file only */
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */
#endif
