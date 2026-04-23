// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#ifndef OpDebug_DEFINED
#error "OpDebug header must precede"
#endif

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "chalkboard" // e.g., "simplify"  one file
#if !OP_DEBUG_ALT
#define TEST_FIRST ""  // "testChalkboard60942"  // e.g., "testLoops40320" if file, one test
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
#define TEST_DEFEAT_DUMPS 1  // set to one to disallow rewriting dumps

// loop191404 missing -0.078, 1.5323 t=0.281543255 oppT=0.290549636; 
//                    -0.3563 2.0153 t=0.139774203 oppT=0.113169670
#define CURVE_CURVE_1 4  // id of segment 1 to break in divide and conquer
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

/* testsRun:3889619 testsSkipped:0  avg pixelError:0.000391925016 maxError:0.083270669 largestError:testCubics2952164
testCubics2952164 error:0.083270669
testCubics2952168 error:0.083270669
testCubics2952172 error:0.083270669
testCubics2952176 error:0.083270669
testCubics1213284 error:0.0832263231
testCubics1213288 error:0.0832263231
testCubics1213292 error:0.0832263231
testCubics1213296 error:0.0832263231
testCubics2952163 error:0.0832172632
testCubics2952167 error:0.0832172632
testCubics2952171 error:0.0832172632
testCubics2952175 error:0.0832172632
testCubics1213283 error:0.0832005739
testCubics1213287 error:0.0832005739
testCubics1213291 error:0.0832005739
testCubics1213295 error:0.0832005739
testCubics3844445 error:0.0829936266
testCubics3844446 error:0.0829936266
testCubics3844447 error:0.0829936266
testCubics3844448 error:0.0829936266
 */

/* testsRun:30046751 testsSkipped:0  avg pixelError:0.000118281409
testQuadralaterals17133255 error:0.000937074423
testQuadralaterals17133256 error:0.000937074423
testQuadralaterals5167255 error:0.000937074423
testQuadralaterals5167256 error:0.000937074423
testQuadralaterals17134615 error:0.000901773572
testQuadralaterals17134616 error:0.000901773572
testQuadralaterals10438615 error:0.000884726644
testQuadralaterals10438616 error:0.000884726644
testQuadralaterals16931704 error:0.000876757316
testQuadralaterals5167204 error:0.000876042061
testQuadralaterals5167257 error:0.00086905621
testQuadralaterals5167258 error:0.00086905621
testQuadralaterals18699160 error:0.000863005174
testQuadralaterals17133166 error:0.000862714835
testQuadralaterals17133270 error:0.000862426125
testQuadralaterals17644888 error:0.000860323198
testQuadralaterals17132768 error:0.00086011854
testQuadralaterals3275768 error:0.000858700601
testQuadralaterals5167388 error:0.000857743435
testQuadralaterals4818416 error:0.000857135281
*/

/* testsRun:3889619 testsSkipped:0  avg pixelError:~0
testRects3119262 error:4.76837158e-07
testRects3119266 error:4.76837158e-07
testRects3119272 error:4.76837158e-07
testRects3119276 error:4.76837158e-07
testRects3120862 error:4.76837158e-07
testRects3120866 error:4.76837158e-07
testRects3120872 error:4.76837158e-07
testRects3120876 error:4.76837158e-07
testRects2731186 error:4.76837158e-07
testRects2731187 error:4.76837158e-07
testRects2731189 error:4.76837158e-07
testRects2731192 error:4.76837158e-07
testRects2731194 error:4.76837158e-07
testRects2731195 error:4.76837158e-07
testRects2731197 error:4.76837158e-07
testRects2731198 error:4.76837158e-07
testRects2732786 error:4.76837158e-07
testRects2732787 error:4.76837158e-07
testRects2732789 error:4.76837158e-07
testRects2732792 error:4.76837158e-07
*/

/* (fast) testsRun:564480 testsSkipped:0 
   (circle) testsRun:564480 testsSkipped:0
*/

/* testsRun:2130976 testsSkipped:0  avg pixelError:0.000106091058
testTriangles1224829 error:0.000731871463
testTriangles1233085 error:0.000716149807
testTriangles1224490 error:0.000715897419
testTriangles1224491 error:0.000715897419
testTriangles370749 error:0.000714466907
testTriangles370757 error:0.000714242458
testTriangles376948 error:0.000705687795
testTriangles376949 error:0.000705687795
testTriangles376971 error:0.000705687795
testTriangles1232819 error:0.000700204168
testTriangles1233090 error:0.000700204168
testTriangles1233091 error:0.000700204168
testTriangles1232746 error:0.000699222088
testTriangles1232747 error:0.000699222088
testTriangles370658 error:0.000692055561
testTriangles370659 error:0.000692055561
testTriangles370778 error:0.000691831112
testTriangles370779 error:0.000691831112
testTriangles1224563 error:0.000691263471
testTriangles1224834 error:0.000691263471
*/

/* testsRun:194480 testsSkipped:0  avg pixelError:0.000483194715 maxError:0.107989788 largestError:testLoops70924
testLoops70924 error:0.107989788
testLoops81487 error:0.0702061653
testLoops32467 error:0.068382144
testLoops77434 error:0.0434149504
testLoops78856 error:0.0431461334
testLoops15846 error:0.040163517
testLoops14901 error:0.0342123508
testLoops136565 error:0.0301455259
testLoops31171 error:0.0274611712
testLoops3822 error:0.0213317871
testLoops163624 error:0.0203206874
testLoops51567 error:0.0200154781
testLoops6867 error:0.0192921162
testLoops62307 error:0.0181521215
testLoops566 error:0.0178154316
testLoops64129 error:0.0177198648
testLoops1970 error:0.0164101124
testLoops542 error:0.0156211592
testLoops136417 error:0.0150725245
testLoops1090 error:0.0144543648
*/
#endif
