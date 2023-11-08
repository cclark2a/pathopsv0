// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 0  // in a debug build: set to zero to enable debug dump, debug image
#define TEST_PATH_OP_FIRST "pentrek4" /* e.g., "battleOp255" test to debug */
#define TEST_PATH_OP_SKIP_TO_FILE "" /* e.g., "battle" tests only (see OpSkiaTests.cpp) */

#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 1  // if 1, skip showing dots, test files started/finished
#define OP_TEST_ALLOW_EXTENDED 0  // some Skia tests have extended versions which take longer
                                  // (out of date) max run: 8,430,493: skipped: 5 error: 335
#define OP_TEST_ENABLE_THREADS 1  // additionally, fast test above must be 1 to use threading
#define OP_MAX_THREADS 16
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions
#define OP_DEBUG_RECORD 0  // track some statistic or other while running

// see descriptions for exceptions below
#define TEST_PATH_OP_EXCEPTIONS ""
#define TEST_PATH_OP_FAIL_EXCEPTIONS ""
#define TEST_PATH_OP_MAP_TO_FUZZ "fuzzhang_1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "" // "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  ""

// !!! need to update laptop exceptions with latest
#define LAPTOP_PATH_OP_EXCEPTIONS ""
#define LAPTOP_PATH_OP_MAP_TO_FUZZ ""
#define LAPTOP_SIMPLIFY_EXCEPTIONS "joel_5"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

/* test failure descriptions:

  last successful run desktop:
total run:735269 skipped:0 errors:1 warnings:26 v0 only:3 skia only:70
  last successful run laptop:
total run:735268 skipped:1 errors:2 warnings:41 v0 only:4 skia only:70

fuzz763_378: no edge found: last, last resort (x2) had errors=36
grshapearc: no edge found: last, last resort (x37) had errors=976
fuzzhang_1: succeeds in skia, fails in v0 (investigate)
tiger8b: no edge found: last, last resort (intermittent, sadly)

!!!  extended tests: (not all fails are listed)
extended test did not complete: only (hundreds of) thread circle tests failed
~5m tests run before first error reported; ~30m tests run in total

thread_circles279395 had errors=13
thread_circles279397 had errors=13
thread_circles279401 had errors=13
thread_circles279403 had errors=13
... (+ 25 more closeby)
thread_circles315826 had errors=87
thread_circles315827 had errors=19
thread_circles315830 had errors=87
thread_circles315829 had errors=19
thread_circles315831 had errors=19
.. (+ 6 more closeby)
thread_circles392738 had errors=1661
thread_circles392742 had errors=1661
thread_circles392746 had errors=1661
thread_circles392750 had errors=1661
thread_circles392758 had errors=1661
.. (+2 more closeby)
thread_circles401474 had errors=65
thread_circles401475 had errors=83
.. (+ 14)
thread_circles414609 had errors=14
thread_circles414610 had errors=2671
.. (+ 10)
thread_circles985585: asserts OpWinder.cpp:302  entry->coincidenceID != coinStart->coincidenceID

laptop only fail:
issue3517 no edge found: last, last resort

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
