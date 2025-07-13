// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "loop" // e.g., "simplify"  one file
#define TEST_FIRST "loop12482"  // "loop70584"  // "loop128079" // "loop171687" // e.g., "testQuads23839519" if file, one test
                        // !!! investigate loop41289 
#define TEST_EXTENDED 1
#define TEST_SKIA 1
#define TEST_REGION 1
#define TEST_ANALYZE 0

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TESTS_TO_SKIP 0 // 14295903  // tests to skip
#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define TEST_DEFEAT_BREAK 0  // set to one to disallow code to stop automatically when running

#define CURVE_CURVE_1 5  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 11  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)

#define TEST_PATH_SKIP_TESTS { "grshapearc", "grshapearcs1", "release_13" }
// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

#endif
