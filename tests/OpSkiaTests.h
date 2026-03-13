// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#ifndef OpDebug_DEFINED
#error "OpDebug header must precede"
#endif

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "quad" // e.g., "simplify"  one file
#if !OP_DEBUG_ALT
#define TEST_FIRST "testQuads18507381"   // e.g., "testQuads5343280" if file, one test
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
#define CURVE_CURVE_1 2  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 6  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)

#define TEST_PATH_SKIP_TESTS { "grshapearc", "grshapearcs1", "release_13", "pentrek10" }
// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST
#define TEST_PATH_OP_SKIP_FILES  /* e.g., "battle", "circleOp" */
// 
/* 6,192,002 run. Those with >= .1 error:
   testQuads6280581 raster errors:0.105732873
....testQuads6281941 raster errors:0.105827905
...testQuads6283061 raster errors:0.105894268
...testQuads6283971 raster errors:0.105965480
..testQuads6284699 raster errors:0.105800003
.testQuads6285271 raster errors:0.105903402
.testQuads6285711 raster errors:0.105895527
.testQuads6286041 raster errors:0.105965883
testQuads6286281 raster errors:0.105880097
testQuads18504901 raster errors:0.110139072
testQuads18506261 raster errors:0.110139072
testQuads18507381 raster errors:0.110139072
...testQuads18508291 raster errors:0.110139072 */
/*  11,620,162 run. Those with >= .1 error:
testQuads6156587 raster errors:0.223082691
testQuads6157947 raster errors:0.223388836
testQuads6159067 raster errors:2.43116117
testQuads6160705 raster errors:0.223260552
testQuads6161277 raster errors:0.223215520
testQuads6161717 raster errors:0.223328352
testQuads6162287 raster errors:0.223132193
testQuads6162455 raster errors:0.223132193
testQuads3279167 raster errors:0.489942312
testQuads3286919 raster errors:0.490059882
testQuads3302423 raster errors:0.326563954
testQuads3310175 raster errors:0.326563954
testQuads3317927 raster errors:0.326681495
testQuads6303675 raster errors:0.219455317
testQuads6305035 raster errors:0.219668731
testQuads6306155 raster errors:0.219499692
testQuads6307065 raster errors:0.219499692
testQuads6307793 raster errors:0.219738066
testQuads6308365 raster errors:0.219584167
testQuads6308805 raster errors:0.219499692 */
#endif
