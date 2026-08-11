// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpSkiaTests_DEFINED
#define OpSkiaTests_DEFINED

#ifndef OpDebug_DEFINED
#error "OpDebug header must precede"
#endif

#define SKIP_TO_V0 0  // set to 1 to ignore file, test first and run first test in v0
#define SKIP_TO_FILE "chalkboard" // e.g.,  "tiger"   one file
#define TEST_SUITE_FIRST ""  // e.g., "simplifyFail" skip suites prior to this one
#if !OP_DEBUG_ALT
#define TEST_FIRST "chalkboard_test_one"   // e.g.,  "tiger8b_h_1"   if file, one test (can be skipped test)
                        // !!! "loop8478" fails sometimes (san/valgrind found no error)
                        // cubic9092  cubic454498  cubic327361 troublesome unsectables
                        // only single tested files get full debugger dumps
#else
#define TEST_FIRST ""  // for debugging two different tests simultaneously (test first & test alt)
#endif
#define TEST_EXTENDED 1

/*
single
conic{{492.451324, 225.216217}, {497.456268, 217.308929}, {497.509460, 217.304092}} weight:0.998576224 isLineSet}
0:{492.423584, 225.115952}, 1:{493.399994, 224.899994}
conic{{0.103891000, 0.00543297734}, {-6.5360074, 6.59994888}, {-6.52924299, 6.65293074}} weight:0.998576224 rotated:yes}
count:1 0.00786512438
double
dconic{{492.451324, 225.216217}, {497.456268, 217.308929}, {497.509460, 217.304092}} weight:0.998576224 isLineSet}
0:{492.423584, 225.115952}, 1:{493.399994, 224.899994}
dconic{{0.103891000, 0.00543297734}, {-6.5360074, 6.59994888}, {-6.52924299, 6.65293074}} weight:0.998576224 rotated:yes}
count:1 0.111898132
*/

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TESTS_TO_SKIP 0  // 21525433  // tests to skip
#define TESTS_TO_RUN 0  // set to zero to run to end (no need to set if 'test first' is set)
#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define USE_DOUBLE_CONICS 0  // set to one to use conics with double calculations intead of float
#define TEST_DEFEAT_BREAK 0  // set to one to disallow debug breakpoints
#define TEST_DEFEAT_DUMPS 0  // set to one to disallow rewriting dumps

// loop191404 missing -0.078, 1.5323 t=0.281543255 oppT=0.290549636; 
//                    -0.3563 2.0153 t=0.139774203 oppT=0.113169670
#define CURVE_CURVE_1 8  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 20  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)
#define CURVE_CURVE_DUMP 0  // 1: dump all ccs; 0: only dump matching curve (for very large tests)
#define UNAMBIGUOUS_DUMP 0  // 1: dump all unamibiguous joins; 0: no dumps (for very large tests)

#define TEST_PATH_SKIP_TESTS { "grshapearcs1" }  /* , "release_13", "pentrek10" */
// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST
#define TEST_PATH_OP_SKIP_FILES  /* e.g., "battle", "circleOp" */
#define TEST_ENABLE_V0 1  // set zero to verify all tests are visited exactly once (disables engine)

#define TEST_ANALYZE 0  // set to one to limit bounds and contours of very large tests
#define LIMIT_CONTOURS 165  // when limited, maximum number of contours to check
#define LIMIT_BOUNDS { 48.0237312, 24.2018681, 50.9992294, 32.9181442 }  // when limited, contours must start in bounds to be checked

/*
trunk:4923 bestGapLimb:[4927 e:823e..870 closeD:0] bestLimb:[4927 e:823e..870 closeD:0] bestDistance:0 bestPerimeter:0.00341796875 maxLimbs:1000 totalUsed:6 limbPass:unsectPair debugAddEach231
[4923 e:826s..869e closeD:0.007055833] parent:- children:[4924 e:870s..823e closeD:0.00461451616] [4925 e:823s..870 closeD:0.00461451616] treePass:linked
[4924 e:870s..823e closeD:0.00461451616] parent:[4923 e:826s..869e closeD:0.007055833] children:[4926 e:824s closeD:0.00261241873] [4927 e:823e..870 closeD:0] treePass:unlinked
[4925 e:823s..870 closeD:0.00461451616] parent:[4923 e:826s..869e closeD:0.007055833] children:[4928 e:870e..823s closeD:0] treePass:unlinked
[4926 e:824s closeD:0.00261241873] parent:[4924 e:870s..823e closeD:0.00461451616] treePass:linked
[4927 e:823e..870 closeD:0] parent:[4924 e:870s..823e closeD:0.00461451616] treePass:unlinked
[4928 e:870e..823s closeD:0] parent:[4925 e:823s..870 closeD:0.00461451616] treePass:unlinked
*/

/* 
testsRun:30046753 testsSkipped:0  avg pixelError:0.00014783004 maxError:0.110236213 largestError:simplifyQuads19830731
simplifyQuads19830731 error:0.110236213
simplifyQuads19845405 error:0.110225432
simplifyQuads19828691 error:0.110217303
simplifyQuads19832411 error:0.110197239
simplifyQuads19851949 error:0.110190123
simplifyQuads19840320 error:0.110185139
simplifyQuads19857034 error:0.110181749
simplifyQuads19842360 error:0.110175960
simplifyQuads19853989 error:0.110165879
simplifyQuads19844040 error:0.110161059
simplifyQuads18747266 error:0.110139072
simplifyQuads18749306 error:0.110139072
simplifyQuads18750986 error:0.110139072
simplifyQuads18752351 error:0.110139072
simplifyQuads19833776 error:0.110139072
simplifyQuads19855669 error:0.110137872
simplifyQuads6576451 error:0.110108651
simplifyQuads6574771 error:0.110087804
simplifyQuads6572731 error:0.110023737
simplifyQuads6428299 error:0.105965883
 */

/* testsRun:3889621 testsSkipped:0  avg pixelError:0.000391115085 maxError:0.0833185911 largestError:opCubics3011319
opCubics3011319 error:0.0833185911
opCubics3011324 error:0.0833185911
opCubics3011329 error:0.0833185911
opCubics3011334 error:0.0833185911
opCubics1226654 error:0.0832742453
opCubics1226659 error:0.0832742453
opCubics1226664 error:0.0832742453
opCubics1226669 error:0.0832742453
opCubics3011318 error:0.0832651854
opCubics3011323 error:0.0832651854
opCubics3011328 error:0.0832651854
opCubics3011333 error:0.0832651854
opCubics1226653 error:0.0832484961
opCubics1226658 error:0.0832484961
opCubics1226663 error:0.0832484961
opCubics1226668 error:0.0832484961
opCubics4195488 error:0.082993865
opCubics4195490 error:0.082993865
opCubics4195492 error:0.082993865
opCubics4195494 error:0.082993865
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

/* testsRun:194482 testsSkipped:0  avg pixelError:0.000478703732 maxError:0.0374162197 largestError:loops83975
loops83975 error:0.0374162197
loops8095 error:0.0213327408
loops2411 error:0.0210089013
loops2455 error:0.0185297187
loops4387 error:0.0164101124
loops1528 error:0.0156211592
loops1576 error:0.0149263041
loops2625 error:0.0144543648
loops1788 error:0.0142819881
loops2839 error:0.0127899647
loops1484 error:0.012759476
loops35172 error:0.0108671188
loops1532 error:0.0108643882
loops3288 error:0.0103959935
loops4215 error:0.0100661041
loops66722 error:0.0099271182
loops601 error:0.00982907694
loops18965 error:0.00976490974
loops21614 error:0.00975561142
loops22497 error:0.00973463058
*/

/* testsRun:594037 testsSkipped:0  avg pixelError:4.0110237e-05 maxError:0.0368652344 largestError:chalkboard10530
chalkboard10530 error:0.0368652344
chalkboard19445 error:0.0368652344
chalkboard22447 error:0.029296875
chalkboard21316 error:0.026550293
chalkboard27071 error:0.026550293
chalkboard24941 error:0.0232543945
chalkboard43130 error:0.0217895508
chalkboard51822 error:0.0201416016
chalkboard124216 error:0.0155639648
chalkboard56939 error:0.0155029297
chalkboard72452 error:0.0155029297
chalkboard36197 error:0.0153808594
chalkboard127884 error:0.0114135742
chalkboard138322 error:0.0114135742
chalkboard152673 error:0.0114135742
chalkboard72683 error:0.0108642578
chalkboard144354 error:0.0108642578
chalkboard166817 error:0.0108642578
chalkboard170611 error:0.0108642578
chalkboard172007 error:0.0108642578
*/

/* simplify
testsRun:464 testsSkipped:0  avg pixelError:0.431449026 maxError:200 largestError:fuzz_twister
fuzz_twister error:200
joel_14x error:0.0277404785
joel_14 error:0.0277404785
cr514118 error:0.0192701649
joel_15x error:0.0191955566
joel_15 error:0.0191955566
bug5169 error:0.0143162562
fuzz763_4713_b error:0.0118160248
joel_4 error:0.00861644745
joel_16x error:0.00304794312
joel_16 error:0.00304794312
carsvg_1 error:0.00259399414
joel_12x error:0.00126647949
joel_12 error:0.00126647949
testQuadratic34 error:0.000606486166
testCubic2 error:0.000540973677
testQuadralateral6ax error:0.000504970551
testQuadralateral6a error:0.000504970551
dean4 error:0.00048828125
testQuadratic97 error:0.000483036041
*/

/* op
testsRun:364 testsSkipped:0 testsError:3 silentError:3  avg pixelError:331.890259 maxError:120808 largestError:crbug_526025
crbug_526025 error:120808
issue2540 error:0.013467595
loops44i error:0.00653188303
loops46i error:0.00506138802
loops45i error:0.00499136001
loops61i error:0.00491813291
loops62i error:0.00489073712
loops5i error:0.00363031775
loops39i error:0.00326216221
loops34i error:0.00321948528
skpbambootheme_com12 error:0.00295233727
bug8380 error:0.00235249475
loops29i error:0.00224936008
issue2753 error:0.0020866394
cubicOp81d error:0.00200282782
loops40i error:0.00172305107
skp5 error:0.00165176392
skpbyte_com1 error:0.00152587891
loops27i error:0.00151711702
cubicOp41i error:0.00142264366
*/

/*
testsRun:639706 testsSkipped:0  avg pixelError:~0
simplifyRect1236402 error:3.81469727e-06
simplifyRect1236403 error:3.81469727e-06
simplifyRect1236404 error:3.81469727e-06
simplifyRect1236405 error:3.81469727e-06
simplifyRect1236406 error:3.81469727e-06
simplifyRect1236407 error:3.81469727e-06
simplifyRect1236408 error:3.81469727e-06
simplifyRect1236409 error:3.81469727e-06
simplifyRect1101505 error:4.76837158e-07
simplifyRect1101506 error:4.76837158e-07
simplifyRect1101515 error:4.76837158e-07
simplifyRect1101516 error:4.76837158e-07
simplifyRect1101525 error:4.76837158e-07
simplifyRect1101526 error:4.76837158e-07
simplifyRect1101535 error:4.76837158e-07
simplifyRect1101536 error:4.76837158e-07
simplifyRect1102255 error:4.76837158e-07
simplifyRect1102256 error:4.76837158e-07
simplifyRect1102265 error:4.76837158e-07
simplifyRect1102266 error:4.76837158e-07
*/

/*
testsRun:700005 testsSkipped:0  avg pixelError:0.00092107855 maxError:0.200589180 largestError:tiger_threaded352557
tiger_threaded352557 error:0.200589180
tiger_threaded353385 error:0.200589180
tiger_threaded353521 error:0.200589180
tiger_threaded354123 error:0.200589180
tiger_threaded354639 error:0.200589180
tiger_threaded356203 error:0.200589180
tiger_threaded356343 error:0.200589180
tiger_threaded359537 error:0.200589180
tiger_threaded361681 error:0.200589180
tiger_threaded362101 error:0.200589180
tiger_threaded362511 error:0.200589180
tiger_threaded362969 error:0.200589180
tiger_threaded363375 error:0.200589180
tiger_threaded363489 error:0.200589180
tiger_threaded363897 error:0.200589180
tiger_threaded364667 error:0.200589180
tiger_threaded365303 error:0.200589180
tiger_threaded366361 error:0.200589180
tiger_threaded366979 error:0.200589180
tiger_threaded367283 error:0.200589180
*/

/*
testsRun:8 testsSkipped:0  avg pixelError:0.00539990328 maxError:0.0141296387 largestError:issue3651_2
issue3651_2 error:0.0141296387
issue3651_5 error:0.00993347168
issue3651_4 error:0.00897216797
issue3651_3 error:0.00796508789
issue3651_7 error:0.00218200684
issue3651_6 error:1.68547358e-05
 */

 /*
 circle totalTests:1270080
testsRun:1270080 testsSkipped:0  avg pixelError:0.000241914735 maxError:0.00638360716 largestError:circles328065
circles328065 error:0.00638360716
circles328068 error:0.00638360716
circles328069 error:0.00638360716
circles328072 error:0.00638360716
circles328073 error:0.00638360716
circles328076 error:0.00638360716
circles328077 error:0.00638360716
circles328080 error:0.00638360716
circles328081 error:0.00638360716
circles328084 error:0.00638360716
circles328085 error:0.00638360716
circles328088 error:0.00638360716
circles328089 error:0.00638360716
circles328092 error:0.00638360716
circles328093 error:0.00638360716
circles328096 error:0.00638360716
circles332097 error:0.00638360716
circles332100 error:0.00638360716
circles332101 error:0.00638360716
circles332104 error:0.00638360716
*/

/*
battle totalTests:381
chalkboard totalTests:594037
fuzz763 totalTests:30
inverse totalTests:320
issue3651 totalTests:8
op totalTests:365
circle totalTests:1270080
....................................................................................................
...................................................................................................1M.
.....................................................
cubic totalTests:3889621
..............................................
...................................................................................................2M.
....................................................................................................
...................................................................................................3M.
....................................................................................................
...................................................................................................4M.
....................................................................................................
...................................................................................................5M.
................................
opFail totalTests:85

Execute debugger commands using "-exec <command>", for example "-exec info registers" will list registers in use (when GDB is the debugger)
fatal error in fuzz763_50

test_drive
*/
#endif
