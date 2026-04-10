// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#ifndef OpDebug_DEFINED
#error "OpDebug header must precede"
#endif

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "cubic" // e.g., "simplify"  one file
#if !OP_DEBUG_ALT
#define TEST_FIRST "testQuads8993401"  // e.g., "testQuads5343280" if file, one test
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

/* testsRun:30046751 testsSkipped:0  avg pixelError:0.00014806533 maxError:0.279829621 largestError:testQuads2325711
testQuads2325711 error:0.279829621
testQuads2356719 error:0.279755592
testQuads2294703 error:0.279751450
testQuads19236931 error:0.110243842
testQuads19227149 error:0.110236213
testQuads19225789 error:0.110217303
testQuads19233541 error:0.110202000
testQuads19228269 error:0.110197239
testQuads19234901 error:0.110191032
testQuads19241293 error:0.110190123
testQuads19244683 error:0.110181749
testQuads19236021 error:0.110167548
testQuads19242653 error:0.110165879
testQuads18504901 error:0.110139072
testQuads18506261 error:0.110139072
testQuads18507381 error:0.110139072
testQuads18508291 error:0.110139072
testQuads19229179 error:0.110139072
testQuads19243773 error:0.110137872
testQuads5383069 error:0.110108651
 */

/* testsRun:3889619 testsSkipped:0  avg pixelError:0.00039643026 maxError:0.136399746 largestError:testCubics1632148
testCubics1632148 error:0.136399746
testCubics1632152 error:0.136399746
testCubics1632156 error:0.136399746
testCubics1632160 error:0.136399746
testCubics970388 error:0.136399746
testCubics970392 error:0.136399746
testCubics970396 error:0.136399746
testCubics970400 error:0.136399746
testCubics1141572 error:0.131867886
testCubics1141576 error:0.131867886
testCubics1141580 error:0.131867886
testCubics1141584 error:0.131867886
testCubics2443972 error:0.131831527
testCubics2443976 error:0.131831527
testCubics2443980 error:0.131831527
testCubics2443984 error:0.131831527
testCubics1904692 error:0.128352165
testCubics1904696 error:0.128352165
testCubics1904700 error:0.128352165
testCubics1904704 error:0.128352165
 */
#endif
