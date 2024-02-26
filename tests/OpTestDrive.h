// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image
#define TEST_PATH_OP_FIRST "thread_cubics1365924"  /* e.g., "battleOp255" (ignored by fast test) */
#define TEST_PATH_OP_SKIP_TO_FILE "cubic" /* e.g., "battle" tests only (see OpSkiaTests.cpp) */
#define TEST_FAILS 0  // set to one to run only failures captured in skia tests

#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_ALLOW_EXTENDED 1  // some Skia tests have extended versions which take longer
                                  // (out of date) max run: 8,430,493: skipped: 5 error: 335
#define OP_TEST_ENABLE_THREADS 1  // additionally, fast test above must be 1 to use threading
#define OP_MAX_THREADS 16
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions
#define OP_DEBUG_RECORD 0  // track some statistic or other while running
#define OP_DEBUG_VERBOSE (1 && !OP_DEBUG_FAST_TEST)

// see descriptions for exceptions below
#define TEST_PATH_OP_EXCEPTIONS "issue3517" // "cubics7d"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1" // "grshapearcs1"
#define TEST_PATH_OP_MAP_TO_FUZZ "fuzzhang_1" // "fuzzhang_1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "" // 
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc" // "grshapearc"
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
fuzz763_378: no edge found: last, last resort (x2) had errors=36 # this looks fixable
             gap between edge 2225 and 2227 (why is 2226 disabled?)
             likely other errors
extended (cubics only):

thread_cubics502920 had errors=27
thread_cubics502924 had errors=27
thread_cubics502916 had errors=27
thread_cubics502928 had errors=27
thread_cubics576817 had errors=15
thread_cubics576819 had errors=15
thread_cubics576820 had errors=28
thread_cubics576823 had errors=15
thread_cubics576824 had errors=28
thread_cubics576821 had errors=15
thread_cubics576825 had errors=15
thread_cubics576828 had errors=28
thread_cubics576827 had errors=15
thread_cubics576829 had errors=15
thread_cubics576831 had errors=15
thread_cubics576832 had errors=28
thread_cubics632484 had errors=44
thread_cubics632488 had errors=44
thread_cubics632492 had errors=44
thread_cubics632496 had errors=44
thread_cubics653330 had errors=19
thread_cubics653334 had errors=19
thread_cubics653335 had errors=25
thread_cubics653338 had errors=19
thread_cubics653339 had errors=25
thread_cubics653331 had errors=25
thread_cubics653343 had errors=25
thread_cubics653342 had errors=19
thread_cubics667283 had errors=60
thread_cubics667286 had errors=60
thread_cubics667282 had errors=60
thread_cubics667290 had errors=60
thread_cubics667287 had errors=26
thread_cubics667291 had errors=60
thread_cubics667294 had errors=60
thread_cubics667295 had errors=41
thread_cubics708850 had errors=414
thread_cubics708854 had errors=414
thread_cubics708862 had errors=414
thread_cubics708858 had errors=414
thread_cubics827604 had errors=90
thread_cubics827612 had errors=90
thread_cubics827603 had errors=90
thread_cubics827608 had errors=90
thread_cubics827607 had errors=90
thread_cubics827611 had errors=90
thread_cubics827615 had errors=90
thread_cubics827616 had errors=90
thread_cubics892945 had errors=107
thread_cubics892953 had errors=107
thread_cubics892949 had errors=107
thread_cubics892957 had errors=107
thread_cubics898001 had errors=48
thread_cubics898003 had errors=90
thread_cubics898004 had errors=90
thread_cubics898012 had errors=90
thread_cubics898011 had errors=90
thread_cubics898009 had errors=48
thread_cubics898007 had errors=90
thread_cubics898005 had errors=48
thread_cubics898008 had errors=90
thread_cubics898013 had errors=48
thread_cubics898015 had errors=90
thread_cubics898016 had errors=90
thread_cubics1210338 no edge found: last, last resort
thread_cubics1210346 no edge found: last, last resort
thread_cubics1210342 no edge found: last, last resort
thread_cubics1210350 no edge found: last, last resort
thread_cubics1225236 no edge found: last, last resort had errors=225
thread_cubics1225240 no edge found: last, last resort had errors=225
thread_cubics1225244 no edge found: last, last resort had errors=225
thread_cubics1225248 no edge found: last, last resort had errors=225

(old)
  last successful run desktop:
total run:735269 skipped:0 errors:1 warnings:26 v0 only:3 skia only:70
  last successful run laptop:
total run:735268 skipped:1 errors:2 warnings:41 v0 only:4 skia only:70

issue3517 no edge found: last, last resort
fuzzhang_1: succeeds in skia, fails in v0 (investigate)

!!!  extended tests: (not all fails are listed)
extended test does not complete: (hundreds of) thread circle, cubic tests fail

75424843 tests run; 467 errors; 56 warnings w/o error

"thread_cubics1648243" OP_ASSERT(lastLink) OpJoiner.cpp:150 

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
.. (+ many more)

thread_cubics157378 had errors=115
thread_cubics157377 had errors=34
thread_cubics157386 had errors=115
.. (+ many more)

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
