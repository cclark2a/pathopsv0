// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

#if defined(__llvm__)
#define OP_HAS_FMA
#else
#define OP_NO_FMA
#endif

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h" // this hasn't been used in a long time and is likely broken
#endif

#define OP_SHOW_TEST_NAME 0
#define OP_TEST_ALLOW_EXTENDED 0
#define OP_TEST_ENABLE_THREADS 1
#define OP_MAX_THREADS 16
#define OP_TEST_V0 1
#define OP_TEST_SKIA 0
#define OP_TEST_REGION 0

// issue3517:  edge 970 and 692 are 1 hex bit from looping, but that isn't detected in matchlinks

// grshapearcs1:  very complicated; defer. Asserts in OpWinder::AddLineCurveIntersection
//                a 'more code needed?' alert that oSegment pinned to bounds

// fuzzhang_1, fuzz763_2674194: succeeds in skia, fails in v0

// testQuadratic75: (simplify) needs lookahead in matchLinks to find best disabled edge
// testQuadratic67x: links unsortable to edge crossing boundary; simple fix breaks loops44i

// battleOp33: A pair of edges are pals; the ends of those edges need to join to connect to next
//             outside edge. There is no edge, disabled or otherwise, that closes the gap. The gap
// also:       is about 6 epsilons wide. Add logic that looks for pals on each edge to know that
// battleOp287 gap can be closed?

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
// also:                 potentially into denormalized values and such. Use OpEpsilon if value is
// thread_circles104487  less than epsilon? Even that wouldn't necessarily save us here.

// thread_rects148587: four coincident lines mixing winding + xor; gets confused
// thread_rects148636: draws incorrectly (debug further)
// thread_rects148651: draws incorrectly (debug further)

// issue3651_2: very large test case; defer til later (fails with unmatched edge in joiner)
//              possible explanation: edge 2489 is in output, but represents two edges (winding 1, 1)
//              edge 2489 is connected to a single edge 2732 (winding 0, 1). Later, edge 2380
//              has nothing to connect to; but it should be able to connect to the unused half of
//              edge 2489

// thread_loops1: cubic is degenerate; asserts in incomplete code OpContour.cpp::64

// fuzz_x1, fuzz_x2: succeeds in skia, fails in v0

#define TEST_PATH_OP_EXCEPTIONS "loops47i", "battleOp33", "battleOp287", \
  "issue3651_2"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1"
#define TEST_PATH_OP_MAP_TO_FUZZ  "fuzzhang_1", "fuzz763_2674194"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "testQuadratic75", "testQuadratic67x", \
  "tiger8b", "tiger8b_x2", "tiger8"
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  "fuzz_x1", "fuzz_x2"

#define LAPTOP_PATH_OP_EXCEPTIONS "issue1417", "fuzz763_378"
#define LAPTOP_PATH_OP_MAP_TO_FUZZ "fuzz763_10022998"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST "thread_circles104483", "thread_loops1"

#define TEST_PATH_OP_FIRST "" /* ""  e.g., "tiger8b_x2" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "" /* "cubic"  e.g., "tiger" to run this file only */
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */
#endif
