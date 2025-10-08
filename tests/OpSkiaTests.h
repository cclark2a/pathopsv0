// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "v0"  // "quad" // e.g., "simplify"  one file
#define TEST_FIRST ""  // e.g., "testQuads23839519" if file, one test
                        // !!! "loop8478" fails sometimes (san/valgrind found no error)
                        // cubic9092  cubic454498  cubic327361 troublesome unsectables
#define TEST_EXTENDED 1
#define TEST_SKIA 1
#define TEST_REGION 1
#define TEST_ANALYZE 0

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TESTS_TO_SKIP 0  // 636082  // tests to skip
#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define TEST_DEFEAT_BREAK 0  // set to one to disallow code to stop automatically when running

// loop191404 missing -0.078, 1.5323 t=0.281543255 oppT=0.290549636; 
//                    -0.3563 2.0153 t=0.139774203 oppT=0.113169670
#define CURVE_CURVE_1 3  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 7  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)

#define TEST_PATH_SKIP_TESTS { "grshapearc", "grshapearcs1", "release_13" }
// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

#endif
