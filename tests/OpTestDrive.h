// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image

#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_TEST_ALLOW_EXTENDED 0  // some Skia tests have extended versions which take longer
#define OP_TEST_ENABLE_THREADS 1  // additionally, fast test above must be 1 to use threading
#define OP_MAX_THREADS 16
#define OP_TEST_V0 1
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions
#define OP_DEBUG_RECORD 0  // track some statistic or other while running

#define TEST_PATH_OP_EXCEPTIONS "loops47i", "battleOp33", "battleOp287", "issue3651_2"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1"
#define TEST_PATH_OP_MAP_TO_FUZZ  "fuzzhang_1", "fuzz763_2674194"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "testQuadratic75", "testQuadratic67x"
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  "fuzz_x1", "fuzz_x2"

#define LAPTOP_PATH_OP_EXCEPTIONS "issue1417", "fuzz763_378"
#define LAPTOP_PATH_OP_MAP_TO_FUZZ "fuzz763_10022998"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST "thread_circles104483"

#define TEST_PATH_OP_FIRST "" /* e.g., "thread_cubics8753" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "" /* e.g., "cubic" tests only (see OpSkiaTests.cpp) */
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

/* test failure descriptions:
grshapearcs1:  very complicated; defer. Asserts in OpWinder::AddLineCurveIntersection
               a 'more code needed?' alert that oSegment pinned to bounds

fuzzhang_1, fuzz763_2674194: succeeds in skia, fails in v0

testQuadratic75: (simplify) needs lookahead in matchLinks to find best disabled edge
testQuadratic67x: links unsortable to edge crossing boundary; simple fix breaks loops44i

battleOp33: A pair of edges are pals; the ends of those edges need to join to connect to next
            outside edge. There is no edge, disabled or otherwise, that closes the gap. The gap
also:       is about 6 epsilons wide. Add logic that looks for pals on each edge to know that
battleOp287 gap can be closed?

loops47i: links edge 546 to 547 (the latter is inside an already-output'd loop)

tiger8: edges 706 and 767 are far apart but separated by a fairly straight line. Along that line
        are many edges, some of which are disabled and others have formed their own loops.
        Solve for now by adding a filler edge to close the gap, rather than failing.
        (emits debug string: "last, last resort")
tiger8b_x2: triggers "last, last" three times (other tiger tests also trigger, but do not fail)

thread_circles104483: misses intersection (temp edges 157, 158) at (0, 1). Suspect that because
(requires extended)   Y-axis answer is close to (is) zero, math falls apart. It certainly points
                      out that using nextafter won't work for numbers near zero, since that falls
also:                 potentially into denormalized values and such. Use OpEpsilon if value is
thread_circles104487  less than epsilon? Even that wouldn't necessarily save us here.

issue3651_2: very large test case; defer til later (fails with unmatched edge in joiner)
             possible explanation: edge 2489 is in output, but represents two edges (winding 1, 1)
             edge 2489 is connected to a single edge 2732 (winding 0, 1). Later, edge 2380
             has nothing to connect to; but it should be able to connect to the unused half of
             edge 2489

fuzz_x1, fuzz_x2: succeeds in skia, fails in v0

thread_cubics532868: triggers assert in OpContour.cpp:382 (debug further) (requires extended)
thread_loops54 and more: draws incorrectly (debug further)
*/

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build (untested)

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h" // this hasn't been used in a long time and is likely broken
#endif

// this isn't used yet, but works and provides an alternative to runningWithFMA()
#if defined(__llvm__)
#define OP_HAS_FMA
#else
#define OP_NO_FMA
#endif

#endif
