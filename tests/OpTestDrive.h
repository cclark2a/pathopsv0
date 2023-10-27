// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image
#define TEST_PATH_OP_FIRST "" /* e.g., "thread_circles194387" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "" /* e.g., "cubic" tests only (see OpSkiaTests.cpp) */

#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_TEST_ALLOW_EXTENDED 0  // some Skia tests have extended versions which take longer
                                  // max run: 8,430,493: skipped: 5 error: 335
#define OP_TEST_ENABLE_THREADS 1  // additionally, fast test above must be 1 to use threading
#define OP_MAX_THREADS 16
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions
#define OP_DEBUG_RECORD 0  // track some statistic or other while running

// see descriptions for exceptions below
#define TEST_PATH_OP_EXCEPTIONS "battleOp287"
#define TEST_PATH_OP_FAIL_EXCEPTIONS ""
#define TEST_PATH_OP_MAP_TO_FUZZ "fuzzhang_1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  ""

// !!! need to update laptop exceptions with latest
#define LAPTOP_PATH_OP_EXCEPTIONS "issue1417", "fuzz763_378"
#define LAPTOP_PATH_OP_MAP_TO_FUZZ "fuzz763_10022998"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

/* test failure descriptions:
total run:735267 skipped:2 errors:12 warnings:21 v0 only:3 skia only:70

grshapearc: hangs in chooseSmallest looping on priorEdge pointing to two edge loop (77719, 77720)
fuzzhang_1: succeeds in skia, fails in v0 (investigate)
battleOp287: addFiller() fails; intersection not found (investigate)
issue3517: no edge found: last, last resort; had errors=22
issue1435: no edge found: last, last resort; had errors=512
fuzz763_378b: no edge found: last, last resort
fuzz763_378: no edge found: last, last resort (x2); had errors=30
issue3651_4: no edge found: last, last resort
issue3651_7: no edge found: last, last resort
issue3651_2: no edge found: last, last resort
thread_cubics78212 had errors=223  (investigate: likely same error as below)
        (was: xor fill makes closed loop out of hole (winding looks fine))
thread_cubics78216: no edge found: last, last resort; had errors=223
thread_cubics78220: no edge found: last, last resort; had errors=223
thread_cubics78224: no edge found: last, last resort; had errors=223
thread_cubics132356: no edge found: last, last resort (x2); had errors=26 (investigate)
thread_cubics132360: no edge found: last, last resort (x2); had errors=26
thread_cubics132364: no edge found: last, last resort (x2); had errors=26
thread_cubics132368: no edge found: last, last resort (x2); had errors=26
thread_loops1658 had errors=73

!!! need to verify that extended still fails as listed: (not all fails are listed)
thread_circles669495: asserts because assemble() failed for a nonfailing test case (extended)
thread_circles1590916: asserts in debugMatchRay() (OpDebug.cpp:423)
                       edge 187 is unsortable, may be why    
thread_cubics532868: triggers assert in OpContour.cpp:382 (debug further) (requires extended)
*/

#define OP_RELEASE_TEST 1	// !!! set to zero to remove tests from release build (untested)

#define OP_DEBUG_COMPARE 0	// set to one to compare successive runs
#if OP_DEBUG_COMPARE
#include "OpDebugCompare.h" // this hasn't been used in a long time and is likely broken
#endif

// an alternative to runningWithFMA()
// only used to attempt to fix debugging on laptop (unsuccessful)
#if defined(__llvm__)
#define OP_HAS_FMA
#else
#define OP_NO_FMA
#endif

#endif
