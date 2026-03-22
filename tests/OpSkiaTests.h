// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#ifndef OpDebug_DEFINED
#error "OpDebug header must precede"
#endif

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "quad" // e.g., "simplify"  one file
#if !OP_DEBUG_ALT
#define TEST_FIRST "testQuads2759881"  // e.g., "testQuads5343280" if file, one test
                        // !!! "loop8478" fails sometimes (san/valgrind found no error)
                        // cubic9092  cubic454498  cubic327361 troublesome unsectables
#else
#define TEST_FIRST "testCubics3251"  // for debugging two different tests simultaneously (test first & test alt)
#endif
#define TEST_EXTENDED 1
#define TEST_ANALYZE 0

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TESTS_TO_SKIP 0  // 893200  // tests to skip
#define TESTS_TO_RUN 0  // set to zero to run to end (no need to set if 'test first' is set)
#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define TEST_DEFEAT_BREAK 0  // set to one to disallow debug breakpoints
#define TEST_DEFEAT_DUMPS 0  // set to one to disallow rewriting dumps

// loop191404 missing -0.078, 1.5323 t=0.281543255 oppT=0.290549636; 
//                    -0.3563 2.0153 t=0.139774203 oppT=0.113169670
#define CURVE_CURVE_1 3  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 8  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)

#define TEST_PATH_SKIP_TESTS { "grshapearc", "grshapearcs1", "release_13", "pentrek10" }
// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST
#define TEST_PATH_OP_SKIP_FILES  /* e.g., "battle", "circleOp" */
// 
/* 26,231,574 run. Those with >= .1 error:
testQuads12894841 raster errors:0.664407015
testQuads12894842 raster errors:0.664477348
testQuads12902593 raster errors:29.2348442
testQuads12902594 raster errors:0.664584875
testQuads12907865 raster errors:0.443082213
testQuads12909225 raster errors:0.664399862
testQuads12918097 raster errors:0.443013191
testQuads12918098 raster errors:0.443102121
testQuads12925849 raster errors:0.443071485
testQuads12925850 raster errors:0.443160415
testQuads12933601 raster errors:19.4893913
testQuads12933602 raster errors:0.443172693
testQuads19263416 raster errors:0.329843998
testQuads7784947 raster errors:0.664374888
testQuads7784948 raster errors:0.664456785
testQuads7792699 raster errors:29.2348518
testQuads7792700 raster errors:0.664448619
testQuads7799091 raster errors:0.664412796
testQuads7815955 raster errors:0.443049401
testQuads7815956 raster errors:0.443145365
testQuads7823707 raster errors:19.4893951
testQuads7823708 raster errors:0.443169892
testQuads2294703 raster errors:0.279744148
testQuads2325711 raster errors:0.279837072
testQuads2356719 raster errors:0.279763043 */
#endif
