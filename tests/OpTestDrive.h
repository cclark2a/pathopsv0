// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTestDrive_DEFINED
#define OpTestDrive_DEFINED

// this puts all switches that decide which tests to run and how to run them in one place

#define OP_DEBUG_FAST_TEST 1  // in a debug build: set to zero to enable debug dump, debug image
#define TEST_PATH_OP_FIRST "testQuadralaterals7644651" // e.g., "thread_cubics2247347" (ignored by fast test)
#define TEST_PATH_OP_SKIP_TO_FILE "simplifyQuadralaterals" // e.g., "battle" tests only (see OpSkiaTests.cpp)

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
#define OP_SHOW_TEST_COUNT 0  // utility to print out test file names and counts for test partitions
#define OP_BULK_THREADS 1  // set to divide all tests among max threads

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
extended: 10650559 quadralateral tests run

testQuadralaterals22710880 had errors=25
"testQuadralaterals2582596" assert in OpJoiner.cpp:191 (OpTree LimbType::miswound)

testQuadralaterals19622648 assert in OpJoiner.cpp:236

testQuads1542819 had errors=65
testQuads1546165 had errors=21
testQuads1550569 had errors=40
testQuads1550591 had errors=31
testQuads1551471 had errors=36
testQuads1551472 had errors=36
testQuads1551551 had errors=36
testQuads1551552 had errors=36
testQuads1551941 had errors=36
testQuads1551942 had errors=36
testQuads1551949 had errors=36
testQuads1551950 had errors=36
testQuads1551955 had errors=36
testQuads1551956 had errors=36
testQuads1551959 had errors=36
testQuads1551960 had errors=36
testQuadralaterals3628783 had errors=496
testQuads1558529 had errors=14
testQuads1558530 had errors=23
testQuads1558641 had errors=11
testQuads1558642 had errors=27
testQuads1559889 had errors=15
testQuads1559890 had errors=19
testQuads1560001 had errors=23
testQuads1560002 had errors=23
testQuads1566095 had errors=31
testQuads1573742 had errors=10
testQuads1573847 had errors=31
testQuads1574153 had errors=46
testQuads1574154 had errors=46
testQuads1575513 had errors=46
testQuads1575514 had errors=46
testQuads1581599 had errors=31
testQuads1582471 had errors=36
testQuads1582472 had errors=36
testQuads1582491 had errors=36
testQuads1582492 had errors=36
testQuads1583551 had errors=29
testQuads6224381 had errors=54
testQuads1589351 had errors=31
testQuads6228513 had errors=213
testQuads6229423 had errors=197
testQuads6232133 had errors=181
testQuads1596998 had errors=10
testQuads6232811 had errors=149
testQuads1597103 had errors=31
testQuads1597409 had errors=63
testQuads1597410 had errors=63
testQuads1598769 had errors=78
testQuads1598770 had errors=78
testQuads1598987 had errors=24
testQuads1598988 had errors=24
testQuads1600107 had errors=24
testQuads1600108 had errors=24
testQuads6238015 had errors=407
testQuads6238587 had errors=250
testQuads6238853 had errors=126
testQuads1604855 had errors=31
testQuads6240143 had errors=126
testQuads15520219 had errors=55
testQuads15520220 had errors=55
testQuads15521579 had errors=46
testQuads15521580 had errors=46
testQuads15522699 had errors=46
testQuads15522700 had errors=46
testQuads1612607 had errors=31
testQuads1614559 had errors=29
testQuads1620359 had errors=31
testQuads1621231 had errors=36
testQuads1621232 had errors=36
testQuads1621251 had errors=36
testQuads1621252 had errors=36
testQuads1622311 had errors=26
testQuads6263743 had errors=33
testQuads6263821 had errors=87
testQuads6263831 had errors=58
testQuads20186683 had errors=36
testQuads20186684 had errors=64
testQuads20186697 had errors=23
testQuads20186698 had errors=72
testQuads20186793 had errors=58
testQuads20186794 had errors=59
testQuads20186999 had errors=12
testQuads20187000 had errors=21
testQuads20188043 had errors=36
testQuads20188044 had errors=52
testQuads20188057 had errors=23
testQuads20188058 had errors=50
testQuads20188153 had errors=62
testQuads20188154 had errors=62
testQuads20188359 had errors=12
testQuads20188360 had errors=14
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
