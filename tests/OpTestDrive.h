// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h" // this hasn't been used in a long time and is likely broken
#endif

#define OP_SHOW_TEST_NAME 0
#define OP_TEST_ALLOW_EXTENDED 01
#define OP_MAX_THREADS 16

// issue3517:  long and skinny; don't know what's going on
//             unterminated ends of contour are edges 1434, (debug further)

// issue issue3651_7: (debug further)

// grshapearcs1:  very complicated; defer. Asserts in OpWinder::AddLineCurveIntersection
//                a 'more code needed?' alert that oSegment pinned to bounds

// fuzzhang_1: succeeds in skia, fails in v0

// fuzz763_2674194: (debug further)

// testQuadratic75: (simplify) needs lookahead in matchLinks to find best disabled edge
// testQuadratic67x: links unsortable to edge crossing boundary; simple fix breaks loops44i

// battleOp33: A pair of edges are pals; the ends of those edges need to join to connect to next
//             outside edge. There is no edge, disabled or otherwise, that closes the gap. The gap
//             is about 6 epsilons wide. Add logic that looks for pals on each edge to know that
//             gap can be closed?

// tiger8b: no matching link found. Edge 547 and 584 are about (3, 13) epsilon apart. The code
//          follows a disabled edge instead of linking them. There is no natural link to use.
//          Edges 547, 584 don't have matching debugRayMatch IDs. Not sure what to do.


// post: use segment for center
// loops47i: links edge 546 to 547 (the latter is inside an already-output'd loop)
// tiger8b_x2: draws incorrectly (debug further)
// tiger_8: fails in matching output edge (debug further)
// tiger8a_1731: fails in addIfUR (requires extended)
//         a tiny edge generates a zero length normal, triggering underflow
//         maybe treating these edges as unsortable will be sufficient
// thread_circles104483: misses intersection (temp edges 157, 158) at (0, 1). Suspect that because
// (requires extended)   Y-axis answer is close to (is) zero, math falls apart. It certainly points
//                       out that using nextafter won't work for numbers near zero, since that falls
//                       potentially into denormalized values and such. Use OpEpsilon if value is
//                       less than epsilon? Even that wouldn't necessarily save us here.

#define TEST_PATH_OP_EXCEPTIONS "issue3517", "loops47i"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "testQuadratic75", "testQuadratic67x", \
  "tiger8b", "tiger8b_x2", "tiger8"
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc"
#define TEST_PATH_OP_MAP_TO_FUZZ  "fuzzhang_1"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST "issue3651_7", "fuzz763_2674194", "battleOp33", 

#define TEST_PATH_OP_FIRST "" /* e.g., "tiger8b_x2" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "" /* e.g., "tiger" to run this file only */
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */
#endif
