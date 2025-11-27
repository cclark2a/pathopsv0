// (c) 2023, Cary Clark cclark2@gmail.com

// !!! test this ! (OpCurve::rawIntersect commented out code)
// for thread_circles104483, edges 113 and 117 fail to find intersection; check for error here

#if OP_DEBUG_FAST_TEST && TEST_ANALYZE
#error "turn off fast test"
#endif

/* test failure descriptions:
extended: all tests run 11/9/24 exceptions: grshapearc (total run:74600014 v0 only:13)

 fuzz763_378 asserts in OpIntersections::sort() debug check line 397 but continuing, succeed

 release_13  increasing limb limit allows it to complete without error

 grshapearc hangs in OpTree contructor? (makes over 10K limbs)
 grshapearc exposed numerous coincident bugs which may be fixed
 TEST_ANALYZE of grshapearc: debugData.limitContours = 165;
 grshapearc next bug is that edge 7831 (first edge in segment 115) crosses segments 83 and 61. Sects in
   115 find coincidence with both 83 and 61 but outside of the range of 7831.
   7831 is detected as inside 7736 (correct) and is sorted outside 7674 (coin pal of 7737) (correct)
   but uses earlier retained inside winding (incorrect). Can the descrepency be detected so that 
   edges involved can be marked as unsectable or unsortable?
   
tests run: 73859160 12/17/2024
 thread_cubics1585153 in "cubic" breaks intersection sort (segment 7) two end coins (opp seg:3)
   sects:[0] id=224 unsect:-276s [2] id:282 coin:330e(!)  [3] id:147 unsect:-177s [4] id:179 unsect:-177e
         [5] id=278 coin:362s unsect:-276e [9] id:280 coin:330e [10] id:225 coin:362e

tests run: 68135597  6/1/2025 fails in tiger8b_x372506 : winding 'setPrior()' recurses on itself
           71733769  6/3/2025 fails in cubic129075 : filler too large
*/

#include "TinySkia.h"
#include "SkiaTestCommon.h"
#include "OpContext.h"  // !!! remove this ?
#include "OpCurve.h"  // !!! remove this ?
#include "OpDebugRaster.h"
#include "OpSkiaTests.h"
#if OP_DEBUG_FAST_TEST
  #include <mutex>
  #include <thread>
  #define OP_MAX_THREADS std::thread::hardware_concurrency()
#else
 #define OP_MAX_THREADS 1
#endif
#include <atomic>

struct testInfo {
    void (*func)(skiatest::Reporter* );
    std::string name;
    int count;  // approximate number is ok; used for thread partitioning
    int extended; 
};

// !!! some tests require counters to track the current test
//      known examples are "cubic" and "simplifyQuadralaterals"
//      identify all and add thread local globals for this
std::vector<testInfo> testSuites = {
    // !!! start out slow
    { run_v0_tests, "v0", 14, 14 },
    { run_op_tests, "op", 451, 451 },
    { run_battle_tests, "battle", 381, 381 },
    { run_chalkboard_tests, "chalkboard", 6231, 594037 },
    { run_fuzz763_tests, "fuzz763", 30, 30 },
    { run_inverse_tests, "inverse", 320, 320 },
    { run_issue3651_tests, "issue3651", 8, 8 },
    { run_op_circle_tests, "circle", 84672, 1778112 },
    { run_op_cubic_tests, "cubic", 148176, 2247347 },
    { run_op_loop_tests, "loop", 9261, 194481 },
    { run_op_rect_tests, "rect", 148176, 3111696 },
    { run_op_fast_tests, "fast", 9216, 82944 },
    { run_simplify_tests, "simplify", 465, 465 },
    { run_simplify_degenerate_tests, "degenerate", 47872, 2345984 },
    { run_simplify_fail_tests, "fail", 7, 7 },
    { run_simplify_quadralaterals_tests, "quadralateral", 124032, 30046752 },
    { run_simplify_quads_tests, "quad", 124032, 30046752 },
    { run_simplify_rect_tests, "rects", 152, 1280660 },
    { run_simplify_triangles_tests, "triangle", 24768, 2130048 },
    { run_tiger_tests, "tiger", 7005, 700005 },
};

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string requestedFirst = TEST_FIRST;
std::string testFirst = OP_DEBUG_FAST_TEST || SKIP_TO_V0 ? "" : TEST_FIRST;
bool runOneFile = !OP_DEBUG_FAST_TEST && (!requestedFirst.empty() || SKIP_TO_V0);
std::string skipToFile = !OP_DEBUG_FAST_TEST && SKIP_TO_V0  ? "v0" : SKIP_TO_FILE;
std::atomic_int testIndex; 
bool showTestName = OP_SHOW_TEST_NAME;
std::atomic_int testsRun;
std::atomic_int testsDot;
std::atomic_int testsLine;
std::atomic_int totalRun;
std::atomic_int testsSkipped;
std::atomic_int testsToSkip;
std::atomic_int totalSkipped;
std::atomic_int silentError;
std::atomic_int totalError;
std::atomic_int treeError;
std::atomic_int gapError;
std::atomic_int testsFailSkiaPass;
std::atomic_int totalFailSkiaPass;
std::atomic_int testsPassSkiaFail;
std::atomic_int totalPassSkiaFail;
#if OP_DEBUG_FAST_TEST
#define OP_THREAD_LOCAL thread_local
#else
#define OP_THREAD_LOCAL
#endif
OP_THREAD_LOCAL std::string currentTestFile;
OP_THREAD_LOCAL int firstSuiteTest = 0;
OP_THREAD_LOCAL int lastSuiteTest = 0;
OP_THREAD_LOCAL int unnamedCount = 0;
OP_THREAD_LOCAL bool needsName = false;
#if OP_DEBUG_FAST_TEST
std::mutex out_mutex;
#endif

bool PathOpsDebug::gCheckForDuplicateNames = false;
bool PathOpsDebug::gJson = false;
// both false if before first; start false end true if no first; both true if after first
bool startFirstTest = "" == testFirst;
bool endFirstTest = false;

// break (return false) if running last failed fast test
#if OP_DEBUG && !OP_DEBUG_FAST_TEST && !OP_TINY_TEST
bool OpDebugSkipBreak() {
	return TEST_DEFEAT_BREAK || (!SKIP_TO_V0 && !requestedFirst.size());
}
#endif

// short-circuit extended if only one test is run
bool skiatest::Reporter::allowExtendedTest() {
    return "" == testFirst ? TEST_EXTENDED : !endFirstTest;
}

// If code is built with the right architecture, the numerics use float multiply-add, which is
// signficantly more accurate that separate multiply and add instructions. Both should work.
// Since many tests may fail with one or the other, detect it, and use it to filter the test list
// accordingly.
bool runningWithFMA() {
    static bool calculated = false;
    static bool runWithFMA = false;
    if (calculated)
        return runWithFMA;
    // this is fragile, but as of 10/04/23, detects FMA with these values.
    LinePts line = { OpPoint(OpDebugBitsToFloat(0x4308f83e), OpDebugBitsToFloat(0x4326aaab)),  // {136.97, 166.667}
        OpPoint(OpDebugBitsToFloat(0x42c55d28), OpDebugBitsToFloat(0x430c5806)) };  // {98.68195, 140.344}
    float adj = line.pts[1].x - line.pts[0].x;
    float opp = line.pts[1].y - line.pts[0].y;
    auto rotatePt = [line, adj, opp](OpPoint pt) {
        OpVector v = pt - line.pts[0];
        return OpPoint(v.dy * adj - v.dx * opp, v.dy * opp + v.dx * adj);
    };
    OpPoint start = rotatePt({ OpDebugBitsToFloat(0x42f16017), OpDebugBitsToFloat(0x431b7908) });
    OP_ASSERT(start.x == OpDebugBitsToFloat(0x390713e0)     // 0.00012882007
            || start.x == OpDebugBitsToFloat(0x39000000));  // 0.00012207031
    runWithFMA = start.x == OpDebugBitsToFloat(0x390713e0);
    calculated = true;
    return runWithFMA;
}

void initializeTests(skiatest::Reporter* r, const char* name) {
    if (r)
        r->testname = name;
}

void initTests(std::string filename) {
    totalRun += testsRun;
    totalSkipped += testsSkipped;
    totalFailSkiaPass += testsFailSkiaPass;
    totalPassSkiaFail += testsPassSkiaFail;
    if (testsRun || testsSkipped || totalRun || totalSkipped)
        OpDebugOut(currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped)
                + " v0:" + STR(testsPassSkiaFail) + " sk:" + STR(testsFailSkiaPass)
                + " total run:" + STR(totalRun) + " skipped:" + STR(totalSkipped)
                + " err:" + STR(totalError)
                + " treeErr:" + STR(treeError)
                + " gapErr:" + STR(gapError)
                + " v0:" + STR(totalPassSkiaFail) + " sk:" + STR(totalFailSkiaPass) + "\n");
    currentTestFile = filename;
    testsRun = 0;
    testsDot = 0;
    testsLine = 0;
    testsSkipped = 0;
    testsToSkip = TESTS_TO_SKIP;
    testsFailSkiaPass = 0;
    testsPassSkiaFail = 0;
    OpDebugOut(currentTestFile + "\n");
}


bool skipTest(std::string name) {
    if (endFirstTest)
        return true;
    if (name != testFirst) {
        if ("" != testFirst)
            return (void) ++testsSkipped, true;
        if (skipRestFiles.end() != std::find(skipRestFiles.begin(), skipRestFiles.end(),
                name))
            skipTestFiles.push_back(currentTestFile);
        if ((skipToFile.size() && currentTestFile != skipToFile) 
                || skipTestFiles.end() != std::find(skipTestFiles.begin(), skipTestFiles.end(), 
                currentTestFile))
            return (void) ++testsSkipped, true;
        if (testsToSkip)
            return (void) --testsToSkip, (void) ++testsSkipped, true;
    }
    if (runOneFile)
        startFirstTest = true;
#if OP_DEBUG_FAST_TEST
    --firstSuiteTest;
    --lastSuiteTest;
    if (firstSuiteTest >= 0 || lastSuiteTest < 0)
        return true;
#endif
    if (showTestName)
        OpDebugOut(name + "\n");
    ++testsRun;
    {
#if OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
#endif
		if (!OP_SHOW_ERRORS_ONLY && !showTestName && testsRun && testsRun % 1000000 == 0)
			OpDebugOut(STR(testsRun / 1000000) + "M");
        ++testsDot;
        ++testsLine;
        if (!OP_SHOW_ERRORS_ONLY && !showTestName 
                && testsDot > (TEST_EXTENDED ? 5000 : 500)) {
            OpDebugOut(".");
            testsDot -= TEST_EXTENDED ? 5000 : 500;
            if (testsLine > (TEST_EXTENDED ? 500000 : 50000)) {
                OpDebugOut("\n");
                testsLine -= TEST_EXTENDED ? 500000 : 50000;
            }
        }
    }
    return false;
}

// if skipToFile is set, run a single test divided among threads
void bulkTest(int index) {
	skiatest::Reporter reporter;
    int totalTests = 0;
    for (auto testSuite : testSuites) {
        if (skipToFile.size() && testSuite.name != skipToFile)
            continue;
        totalTests += TEST_EXTENDED ? testSuite.extended : testSuite.count;
    }
    int firstTest = index * totalTests / OP_MAX_THREADS;
    firstTest += TESTS_TO_SKIP / OP_MAX_THREADS;
    int lastTest = (index + 1) * totalTests / OP_MAX_THREADS;
    for (auto testSuite : testSuites) {
        if (skipToFile.size() && testSuite.name != skipToFile)
            continue;
        int testCount = TEST_EXTENDED ? testSuite.extended : testSuite.count;
        firstTest -= testCount;
        lastTest -= testCount;
        if (firstTest < 0) {
            firstSuiteTest = testCount + firstTest;
            lastSuiteTest = testCount + lastTest;
            currentTestFile = testSuite.name;
            needsName = testSuite.extended != testSuite.count;
            unnamedCount = 0;
			reporter.filename = testSuite.name;
			reporter.testname = "";
            (testSuite.func)(&reporter);
            if (lastTest <= 0)
                return;
            firstTest = 0;
        }
    }
}

uint64_t timerFrequency;
uint64_t timerStart;

void runTests() {
    timerFrequency = OpInitTimer();
    timerStart = OpReadTimer();
#if OP_DEBUG_FAST_TEST
    std::vector<std::thread> t;
    for (unsigned index = 0; index < OP_MAX_THREADS; ++index)
        t.push_back(std::thread(bulkTest, index));
    for (unsigned index = 0; index < OP_MAX_THREADS; ++index)
        t[index].join();
#else
	skiatest::Reporter reporter;
    auto runTest = [&reporter](std::string s) {
        for (auto suite : testSuites) {
            if (suite.name == s) {
                currentTestFile = suite.name;
                needsName = suite.extended != suite.count;
                unnamedCount = 0;
                initTests(suite.name);
				reporter.filename = suite.name;
                (*suite.func)(&reporter);
                return;
            }
        }
    };
    if (skipToFile.size()) {
        runTest(skipToFile);
    } else {
        runTest("v0");  // check for these failures first
        runTest("op");
        for (auto pair : testSuites) {
            if (pair.name != "v0" && pair.name != "op")
                runTest(pair.name);
        }
    }
#endif
    uint64_t end = OpReadTimer();
    float elapsed = OpTicksToSeconds(end - timerStart, timerFrequency);
#if OP_DEBUG_FAST_TEST
    OpDebugOut("skia tests done: " + STR(elapsed) + "s\n");
#else
    initTests("skia tests done: " + STR(elapsed) + "s\n");
#endif
    if (testsRun || testsSkipped)
        OpDebugOut("total run:" + STR(testsRun) + " skipped:" + STR(testsSkipped) 
            + " errors:" + STR(totalError)
            + " treeErr:" + STR(treeError)
            + " gapErr:" + STR(gapError)
            + " v0 only:" + STR(testsPassSkiaFail) + " skia only:" + STR(testsFailSkiaPass) + "\n");
}

void ReportError(std::string testname, int errors) {
    std::string s = testname;
    if (errors)
        s += " had errors=" + STR(errors);
    OpDebugOut(s + "\n");
    OpDebugOut("");  // for setting a breakpoint
}

#include "port/SkiaPaths.h"
#include "curves/BinaryWinding.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

// char* so it can be called from immediate window
#if !OP_DEBUG_FAST_TEST
void dumpOpTest(const char* testname, const SkPath& pathA, const SkPath& pathB, SkPathOp op) {
    std::string filename = dmpFileToPath(TestFile);
    FILE* file = fopen(filename.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filename + " to write\n");
        return;
    }
	std::string s;
    s += "void ";
    s += testname;
    s += "(skiatest::Reporter* reporter, const char* filename) {\n";
    s += "    SkPath pathA, path;\n";
    s += dumpSkPath(&pathA, false) + "\n";
    s += "    pathA = path;\n";
    s += "    path.reset();\n";
    s += dumpSkPath(&pathB, false) + "\n";
    std::string opStr;
    switch(op) {
        case SkPathOp::kDifference_SkPathOp: opStr = "SkPathOp::kDifference_SkPathOp"; break;
        case SkPathOp::kIntersect_SkPathOp: opStr = "SkPathOp::kIntersect_SkPathOp"; break;
        case SkPathOp::kUnion_SkPathOp: opStr = "SkPathOp::kUnion_SkPathOp"; break;
        case SkPathOp::kXOR_SkPathOp: opStr = "SkPathOp::kXOR_SkPathOp"; break;
        case SkPathOp::kReverseDifference_SkPathOp: opStr = "SkPathOp::kReverseDifference_SkPathOp"; break;
        default: OP_ASSERT(0);
    }
    s += "    testPathOp(reporter, pathA, path, " + opStr + ", filename);\n";
    s += "}\n\n";
    s += "static struct TestDesc tests[] = {\n";
    s += "    TEST(" + std::string(testname) + "),\n";
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}
#endif

void trackError(PathOpsV0Lib::ContextError contextError) {
	if (PathOpsV0Lib::ContextError::none != contextError)
		++totalError;
	switch (contextError) {
		case PathOpsV0Lib::ContextError::none:
			break;
		case PathOpsV0Lib::ContextError::finite:  // input was not finite
			++silentError; 
			break;
		case PathOpsV0Lib::ContextError::toVertical:  // skewing curve exceeds float range
			++silentError; 
			break;
		case PathOpsV0Lib::ContextError::tree:
			++treeError;
			break;
		case PathOpsV0Lib::ContextError::gap:
			++gapError;
			break;
        case PathOpsV0Lib::ContextError::root:
            ++silentError;
            break;
		default:
			OP_ASSERT(0);
	}
}

#if OP_TINY_SKIA
extern void alt_cubicOp130a();
extern void alt_loop1asQuad();
extern void alt_testArc();
#endif
extern void alt_loops58iAsQuads();
extern void alt_loops59iasQuads();
extern void alt_loops33iAsQuads();
extern void alt_loops40iAsQuads();
extern void alt_cubicOp114asQuad();

bool OpV0(const SkPath& a, const SkPath& b, SkPathOp op, SkPath* result,
		OpDebugData* debugDataPtr) {
    using namespace PathOpsV0Lib;
    Context* context = CreateContext();
    ContextUserData data { result, sizeof(result), UserDataType::outPath };
    AddUserData(context, data);
    SetSkiaContextCallbacks(context);
    OP_DEBUG_CODE(if (debugDataPtr) SetDebugData(context, *debugDataPtr));
    SetSkiaCurveCallbacks(context);
    SkPathOp mappedOp = MapInvertedSkPathOp(op, a.isInverseFillType(), b.isInverseFillType());
    auto isWindingFill = [](const SkPath& path) {
        return SkPathFillType::kWinding == path.getFillType()
                || SkPathFillType::kInverseWinding == path.getFillType();
    }; 
    bool aIsWinding = isWindingFill(a);
    bool bIsWinding = isWindingFill(b);
    BinaryWindType windType = aIsWinding && bIsWinding ? BinaryWindType::windBoth
            : aIsWinding ? BinaryWindType::windLeft : bIsWinding ? BinaryWindType::windRight
            : BinaryWindType::evenOdd;
	SetSkiaOpContextCallbacks(context, mappedOp, windType);
    int leftData[] = { 1, 0 };
    Contour* left = SetSkiaOpContourCallbacks(context, leftData, sizeof(leftData), BinaryOperand::left
            OP_DEBUG_PARAMS(&a));
 //   OP_DEBUG_CODE(BinaryContour debugLeftData { { &a }, BinaryOperand::left });
 //   OP_DEBUG_CODE(AddDebugContour debugLeft { debugLeftData, sizeof(BinaryContour), 
 //           PathOpsV0Lib::DebugContourType::windingUserData } );
    AddSkiaPath(context, left, a  /* OP_DEBUG_PARAMS(&debugLeft) */);
    int rightData[] = { 0, 1 };
    Contour* right = SetSkiaOpContourCallbacks(context, rightData, sizeof(rightData), BinaryOperand::right
            OP_DEBUG_PARAMS(&b));
//    OP_DEBUG_CODE(BinaryContour debugRightData { { &a }, BinaryOperand::right });
//    OP_DEBUG_CODE(AddDebugContour debugRight { debugRightData, sizeof(BinaryContour), 
//            PathOpsV0Lib::DebugContourType::windingUserData } );
    AddSkiaPath(context, right, b  /* OP_DEBUG_PARAMS(&debugRight) */);
#if TEST_RASTER
    DebugRaster debugRaster((OpContext*) context);
    if (OpDebugExpect::success == debugDataPtr->expect)
        debugRaster.in();
#endif
    Resolve(context);
    if (runOneFile)
        dmpFile();
    if (SkPathOpInvertOutput(op, a.isInverseFillType(), b.isInverseFillType()))
        result->toggleInverseFillType();
    ContextError contextError = Error(context);
	trackError(contextError);
#if TEST_RASTER
    if (OpDebugExpect::success == debugDataPtr->expect)
        /* float compareError = */ debugRaster.out();
#endif
    DeleteContext(context);
	return ContextError::none == contextError;
}

// mayDiffer is true if test is fuzz with large values that Skia ignores
void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail, bool mayDiffer) {
	auto alt = [&testname](std::string name, void (*func)()) {
		if (name == testname) {
			(*func)();
			return true;
		}
		std::string altname = "alt_" + name;
		if (altname == testname)
			testname = name;
		return false;
	};
#if OP_TINY_SKIA
	if (alt("cubicOp130a", alt_cubicOp130a))
		return;
	if (alt("loop1asQuad", alt_loop1asQuad))
		return;
#endif
	if (alt("loops58iAsQuads", alt_loops58iAsQuads))
		return;
	if (alt("loops59iasQuads", alt_loops59iasQuads))
		return;
	if (alt("loops33iAsQuads", alt_loops33iAsQuads))
		return;
	if (alt("loops40iAsQuads", alt_loops40iAsQuads))
		return;
	if (alt("cubicOp114asQuad", alt_cubicOp114asQuad))
		return;
    const char* tn = testname.c_str();
    SkPath result;
    result.setFillType(SkPathFillType::kEvenOdd);  // !!! workaround
    OpDebugData debugData(v0MayFail);
    debugData.testname = testname;
    debugData.curveCurve1 = CURVE_CURVE_1;
    debugData.curveCurve2 = CURVE_CURVE_2;
    debugData.curveCurveDepth = CURVE_CURVE_DEPTH;
	debugData.runOneFile = runOneFile || SKIP_TO_V0;
#if !OP_DEBUG_FAST_TEST
    dumpOpTest(tn, a, b, op);
#endif
	(void) OpV0(a, b, op, &result, &debugData);
#if TEST_SKIA
    SkPath skresult;
	bool skSuccess = skiaMayFail;
#if OP_TEST_V0
    if (debugData.success && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#elif TEST_REGION
    bool skSuccess = true;
#endif
    if (startFirstTest && runOneFile)
        endFirstTest = true;
#if OP_TEST_V0 && TEST_REGION
    if (!debugData.success || !skSuccess || v0MayFail || skiaMayFail || mayDiffer)
        return;
    int errors = (int) (debugData.error + .5f);
//  int altErrors = VerifyOpNoRegion(a, b, op, result);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
#endif
        ReportError(testname, errors);
        if (errors > MAX_ERRORS)
            totalError++;
#endif
    }
#else
	if (std::string(tn) == "never!")  // prevent tn from being optimized out
		OpDebugOut("");
#endif
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* name, bool v0MayFail, bool skiaMayFail, bool mayDiffer) {
    if (skipTest(name)) {
        return true;
	}
    threadablePathOpTest(0, a, b, op, name, v0MayFail, skiaMayFail, mayDiffer);
    return true;
}

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testname) {
	std::string s(testname);
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    }
    return testPathOpBase(r, a, b, op, testname, false, false, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname, bool checkFail) {
    testPathOpBase(r, a, b, op, testname, false, false, true);
}

void testPathOpFuzz(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname) {
	std::string s(testname);
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    }
    testPathOpBase(r, a, b, op, testname, true, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    return testPathOpBase(r, a, b, op, testName, false, true, true);
}

void RunTestSet(skiatest::Reporter* r, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter* , const char* testName),
        void (*skipTest)(skiatest::Reporter* , const char* testName),
        void (*stopTest)(skiatest::Reporter* , const char* testName), bool reverse) {
	static const std::vector<std::string> skipTests = TEST_PATH_SKIP_TESTS;
    for (size_t i = 0; i < count; ++i) {
		r->testname = tests[i].str;
		if (skipTests.end() != std::find(skipTests.begin(), skipTests.end(), r->testname))
			continue;
        (*tests[i].fun)(r, tests[i].str);
	}
}

#if 0
int VerifySimplify(const SkPath& one, std::string testname, const SkPath& result, bool v0mayFail) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, openClip;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnA.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, nullptr, scale);
    SkRegion scaledRgnA;
    SkPath scaledA;
    scaledA.addPath(one, scale);
    scaledA.setFillType(one.getFillType());
    scaledRgnA.setPath(scaledA, openClip);
    scaledRgnA.getBoundaryPath(&scaledPathOut);
    SkBitmap bitmap;
    SkPath scaledOut;
    scaledOut.addPath(result, scale);
    scaledOut.setFillType(result.getFillType());
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap, v0mayFail, false);
    return errors;
}
#endif

#if !OP_DEBUG_FAST_TEST
std::string debugSimplifyTest(const char* testname, const SkPath& path) {
	std::string s;
    s += "void " + STR(testname) + "(skiatest::Reporter* reporter, const char* filename) {\n";
    s += "    SkPath path;\n";
	s += dumpSkPath(&path, false) + "\n";
	s += "    testSimplify(reporter, path, filename);\n";
    s += "}\n\n";
    s += "static struct TestDesc tests[] = {\n";
    s += "    TEST(" + std::string(testname) + "),\n";
	return s;
}

// char* so it can be called from immediate window
void dumpSimplifyTest(const char* testname, const SkPath& path) {
    std::string filename = dmpFileToPath(TestFile);
    FILE* file = fopen(filename.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filename + " to write\n");
        return;
    }
	std::string s = debugSimplifyTest(testname, path);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}
#endif

struct AutoClose {
    AutoClose(FILE* f)
		: file(f) {}
    ~AutoClose() {
		fclose(file); }

    FILE* file;
};

struct AutoFree {
	AutoFree(void* b)
		: buffer((char*) b) {}
	~AutoFree() {
		free(buffer); }

	char* buffer;
};

static void edit(std::string filename, std::string match, std::string replace) {
	std::string directory = "../../../example/";  //e.g., D:\gerrit\skia\out\Debug\obj
	std::string readName = directory + filename;
	AutoClose readf(fopen(readName.c_str(), "rb"));
	if (!readf.file)
		return OpDebugOut("could not read " + readName + "\n");
	if (fseek(readf.file, 0 , SEEK_END))
		return OpDebugOut("fseek to end failed:" + filename + "\n");
	long fileSize = ftell(readf.file);
	long allocSize = (long) (fileSize + replace.size());
	if (fseek(readf.file, 0 , SEEK_SET))
		return OpDebugOut("fseek to start failed:" + filename + "\n");
 	AutoFree read(malloc(allocSize));
	if (!read.buffer)
		return OpDebugOut("malloc failed:" + readName + "; size:" + STR((size_t) allocSize) + "\n");
	size_t bytesRead = fread(read.buffer, 1, fileSize, readf.file);
	if (bytesRead < (size_t) fileSize)
		return OpDebugOut("read failed:" + readName + "; read:" + STR(bytesRead) 
				+ " expected:" + STR((size_t) fileSize) + "\n");
	std::string writeName = directory + "temp";
	AutoClose write(fopen(writeName.c_str(), "wb"));
	if (!write.file)
		return OpDebugOut("could not open " + writeName + "\n");
	char* matchPos = strstr(read.buffer, match.c_str());
	if (!matchPos)
		return OpDebugOut("no match in:" + filename + " for:" + match + "\n");
	ptrdiff_t startSize = matchPos - read.buffer;
	size_t bytesWritten = fwrite(read.buffer, 1, startSize, write.file);
	if (bytesWritten != (size_t) startSize)
		return OpDebugOut("write start failed:" + filename + "; written:" + STR(bytesWritten)
				+ " expected:" + STR((size_t) startSize) + "\n:");
	bytesWritten = fwrite(replace.c_str(), 1, replace.size(), write.file);
	if (bytesWritten != replace.size())
		return OpDebugOut("write replace failed:" + filename + "; written:" + STR(bytesWritten)
				+ " expected:" + STR(replace.size()) + "\n:");
	size_t endPos = startSize + match.size();
	bytesWritten = fwrite(read.buffer + endPos, 1, fileSize - endPos, write.file);
	if (bytesWritten < fileSize - endPos)
		return OpDebugOut("write failed:" + writeName + "; written:" + STR(bytesWritten) 
				+ " expected:" + STR((int) (fileSize - endPos)) + "\n");
	remove(readName.c_str());
	rename(writeName.c_str(), readName.c_str());
}

void v0(const char* testname, const SkPath& path) {
#if 01 && defined _WIN32
   char full[_MAX_PATH];
   if( _fullpath( full, ".\\", _MAX_PATH ) != NULL )
      OpDebugOut( "Full path is: " + std::string(full) + "\n");
   else
      OpDebugOut( "Invalid path\n" );
#endif
	edit("tests/OpTestDrive.h", "#define OP_DEBUG_FAST_TEST 1", "#define OP_DEBUG_FAST_TEST 0");
	std::string addedTest = debugSimplifyTest(testname, path);
	edit("tests/OpV0Tests.cpp", "static struct TestDesc tests[] = {", addedTest);
}

void run() {
	edit("tests/OpTestDrive.h", "#define OP_DEBUG_FAST_TEST 0", "#define OP_DEBUG_FAST_TEST 1");
}

bool SimplifyV0(const SkPath& path, SkPath* out, OpDebugData* optional) {
    using namespace PathOpsV0Lib;
    Context* context = CreateContext();
    ContextUserData data { out, sizeof(out), UserDataType::outPath };
    AddUserData(context, data);
    OP_DEBUG_CODE(if (optional) SetDebugData(context, *optional));
    SetSkiaContextCallbacks(context);
    SetSkiaCurveCallbacks(context);
    auto isWindingFill = [](const SkPath& path) {
        return SkPathFillType::kWinding == path.getFillType()
                || SkPathFillType::kInverseWinding == path.getFillType();
    }; 
    int simpleData[] = { 1 };
    Contour* simple = SetSkiaSimplifyCallbacks(context, simpleData, sizeof(simpleData), isWindingFill(path)
            OP_DEBUG_PARAMS(&path));
    OP_DEBUG_CODE(UnaryContour debugData { &path } );
#if TEST_ANALYZE && OP_DEBUG
	// make failing tests smaller
	// add contours until it fails
    AddDebugSkiaPath(context, simple, path  OP_DEBUG_PARAMS(debugData, sizeof debugData));
#else
//    OP_DEBUG_CODE(AddDebugContour debugContour { debugData, sizeof debugData, 
//            DebugContourType::windingUserData } );
    AddSkiaPath(context, simple, path  /* OP_DEBUG_PARAMS(&debugContour) */);
#endif
#if TEST_RASTER
    DebugRaster debugRaster((OpContext*) context);
    if (OpDebugExpect::success == optional->expect)    
        debugRaster.in();
#endif
	ContextError contextError = Error(context);
	bool veryLarge = false;
	if (ContextError::finite == contextError) {
		// 'fail' tests 'dontFailOne' have 1e38 values as input. Generating extrema in skia adapter
		// creates inf and nan. Return if this is the case so that caller can skip reporting error.
		veryLarge = VeryLargeSkiaPath(path);
		if (veryLarge)
			out->setFillType(SkPathFillType::kEvenOdd);
	}
	if (ContextError::none == contextError) {
		Resolve(context);
        if (runOneFile)
            dmpFile();
		contextError = Error(context);
		if (ContextError::toVertical == contextError)
			veryLarge = VeryLargeSkiaPath(path);			
		trackError(contextError);
	}
#if TEST_RASTER
    if (OpDebugExpect::success == optional->expect)
        /* float compareError = */ debugRaster.out();
#endif
#if TEST_ANALYZE && OP_DEBUG
	if (optional) {
		OP_DEBUG_CODE(*optional = ((OpContext*) context)->debugData);
		OP_ASSERT(!optional->limitReached);
	}
#endif
    DeleteContext(context);
	return ContextError::none == contextError || veryLarge;
}

void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail) {
	auto alt = [&testname](std::string name, void (*func)()) {
		if (name == testname) {
			(*func)();
			return true;
		}
		std::string altname = "alt_" + name;
		if (altname == testname)
			testname = name;
		return false;
	};
	if (alt("testArc", alt_testArc))
		return;
    out.setFillType(SkPathFillType::kEvenOdd); // !!! workaround
    const char* tn = testname.c_str();
    if ("never!" == testname)
        OpDebugOut(tn);  // prevent optimizer from removing tn
    OpDebugData debugData(v0MayFail);
    debugData.testname = testname;
    debugData.curveCurve1 = CURVE_CURVE_1;
    debugData.curveCurve2 = CURVE_CURVE_2;
    debugData.curveCurveDepth = CURVE_CURVE_DEPTH;
	debugData.runOneFile = runOneFile || SKIP_TO_V0;
#if TEST_ANALYZE
	debugData.limitContours = 165;
	debugData.limitReached = false;
	int testDots = 0;
	do {
		(void) SimplifyV0(path, &out, &debugData);
		OpDebugOut(".");
		if (++testDots == 100) {
			testDots = 0;
			OpDebugOut("\n");
		}
		++debugData.limitContours;
	} while (!debugData.limitReached);
#else
    dumpSimplifyTest(tn, path);   
	(void) SimplifyV0(path, &out, &debugData);
#endif
    OP_ASSERT(v0MayFail || debugData.success);
#if TEST_SKIA
    SkPath skOut;
	bool skSuccess = skiaMayFail;
    if (debugData.success && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.success && skSuccess)
        testsFailSkiaPass++;
#endif
    if (startFirstTest && runOneFile)
        endFirstTest = true;
    if (!debugData.success)
        return;
    int errors = (int) (debugData.error + .5f);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS && !v0MayFail) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if 0 && OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
		v0(tn, p);
#endif
        ReportError(testname, errors);
#endif
        totalError++;
    }
}

bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& state, 
        const char* name) {
    std::string testname(name);
    if ("" == testname)
        testname = state.fReporter->testname + STR(++unnamedCount);
    if (skipTest(testname)) {
        return true;
	}
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
    threadableSimplifyTest(0, path, testname.c_str(), out, false, false);
    return true;
}

bool testSimplifyBase(skiatest::Reporter* r, const SkPath& path, const char* name, 
        bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name)) {
        return true;
	}
    SkPath out;
    threadableSimplifyTest(0, path, name, out, v0MayFail, skiaMayFail);
    return true;
}

bool testSimplify(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, false, false);
}

bool testSimplifyFail(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, true, true);
}

bool testSimplifyFuzz(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, true, true);
}

PathOpsThreadedRunnable** DebugOneShot::append() {
    if (slot)
        (*slot->fun)(&data);
    delete slot;
    return &slot;
}

PathOpsThreadedRunnable::PathOpsThreadedRunnable(void (*f)(PathOpsThreadState *),
            int a, int b, int c, int d, PathOpsThreadedTestRunner* runner) {
    fun = f;
    runner->fRunnables.data.fA = a;
    runner->fRunnables.data.fB = b;
    runner->fRunnables.data.fC = c;
    runner->fRunnables.data.fD = d;
}

PathOpsThreadedTestRunner::PathOpsThreadedTestRunner(skiatest::Reporter* r) {
    fRunnables.slot = nullptr;
    fRunnables.data.fReporter = r;
}

void PathOpsThreadedTestRunner::render() {
    fRunnables.append();
}

#if 0
// only used to extract test data so tests can run without Skia internal access
#include "src/core/SkPathPriv.h"
#include "src/core/SkTSort.h"
#include "src/pathops/SkPathOpsBounds.h"
#include "src/pathops/SkPathOpsConic.h"
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsLine.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "src/pathops/SkPathOpsTSect.h"
#include "src/pathops/SkReduceOrder.h"
#include "tests/PathOpsTestCommon.h"

#include <utility>

static double calc_t_div(const SkDCubic& cubic, double precision, double start) {
    const double adjust = sqrt(3.) / 36;
    SkDCubic sub;
    const SkDCubic* cPtr;
    if (start == 0) {
        cPtr = &cubic;
    } else {
        // OPTIMIZE: special-case half-split ?
        sub = cubic.subDivide(start, 1);
        cPtr = &sub;
    }
    const SkDCubic& c = *cPtr;
    double dx = c[3].fX - 3 * (c[2].fX - c[1].fX) - c[0].fX;
    double dy = c[3].fY - 3 * (c[2].fY - c[1].fY) - c[0].fY;
    double dist = sqrt(dx * dx + dy * dy);
    double tDiv3 = precision / (adjust * dist);
    double t = SkDCubeRoot(tDiv3);
    if (start > 0) {
        t = start + (1 - start) * t;
    }
    return t;
}

static bool add_simple_ts(const SkDCubic& cubic, double precision, SkTArray<double, true>* ts) {
    double tDiv = calc_t_div(cubic, precision, 0);
    if (tDiv >= 1) {
        return true;
    }
    if (tDiv >= 0.5) {
        ts->push_back(0.5);
        return true;
    }
    return false;
}

static void addTs(const SkDCubic& cubic, double precision, double start, double end,
        SkTArray<double, true>* ts) {
    double tDiv = calc_t_div(cubic, precision, 0);
    double parts = ceil(1.0 / tDiv);
    for (double index = 0; index < parts; ++index) {
        double newT = start + (index / parts) * (end - start);
        if (newT > 0 && newT < 1) {
            ts->push_back(newT);
        }
    }
}

static void toQuadraticTs(const SkDCubic* cubic, double precision, SkTArray<double, true>* ts) {
    SkReduceOrder reducer;
    int order = reducer.reduce(*cubic, SkReduceOrder::kAllow_Quadratics);
    if (order < 3) {
        return;
    }
    double inflectT[5];
    int inflections = cubic->findInflections(inflectT);
    SkASSERT(inflections <= 2);
    if (!cubic->endsAreExtremaInXOrY()) {
        inflections += cubic->findMaxCurvature(&inflectT[inflections]);
        SkASSERT(inflections <= 5);
    }
    SkTQSort<double>(inflectT, inflectT + inflections);
    // OPTIMIZATION: is this filtering common enough that it needs to be pulled out into its
    // own subroutine?
    while (inflections && approximately_less_than_zero(inflectT[0])) {
        memmove(inflectT, &inflectT[1], sizeof(inflectT[0]) * --inflections);
    }
    int start = 0;
    int next = 1;
    while (next < inflections) {
        if (!approximately_equal(inflectT[start], inflectT[next])) {
            ++start;
        ++next;
            continue;
        }
        memmove(&inflectT[start], &inflectT[next], sizeof(inflectT[0]) * (--inflections - start));
    }

    while (inflections && approximately_greater_than_one(inflectT[inflections - 1])) {
        --inflections;
    }
    SkDCubicPair pair;
    if (inflections == 1) {
        pair = cubic->chopAt(inflectT[0]);
        int orderP1 = reducer.reduce(pair.first(), SkReduceOrder::kNo_Quadratics);
        if (orderP1 < 2) {
            --inflections;
        } else {
            int orderP2 = reducer.reduce(pair.second(), SkReduceOrder::kNo_Quadratics);
            if (orderP2 < 2) {
                --inflections;
            }
        }
    }
    if (inflections == 0 && add_simple_ts(*cubic, precision, ts)) {
        return;
    }
    if (inflections == 1) {
        pair = cubic->chopAt(inflectT[0]);
        addTs(pair.first(), precision, 0, inflectT[0], ts);
        addTs(pair.second(), precision, inflectT[0], 1, ts);
        return;
    }
    if (inflections > 1) {
        SkDCubic part = cubic->subDivide(0, inflectT[0]);
        addTs(part, precision, 0, inflectT[0], ts);
        int last = inflections - 1;
        for (int idx = 0; idx < last; ++idx) {
            part = cubic->subDivide(inflectT[idx], inflectT[idx + 1]);
            addTs(part, precision, inflectT[idx], inflectT[idx + 1], ts);
        }
        part = cubic->subDivide(inflectT[last], 1);
        addTs(part, precision, inflectT[last], 1, ts);
        return;
    }
    addTs(*cubic, precision, 0, 1, ts);
}

void CubicToQuads(const SkDCubic& cubic, double precision, SkTArray<SkDQuad, true>& quads) {
    SkTArray<double, true> ts;
    toQuadraticTs(&cubic, precision, &ts);
    if (ts.count() <= 0) {
        SkDQuad quad = cubic.toQuad();
        quads.push_back(quad);
        return;
    }
    double tStart = 0;
    for (int i1 = 0; i1 <= ts.count(); ++i1) {
        const double tEnd = i1 < ts.count() ? ts[i1] : 1;
        SkDRect bounds;
        bounds.setBounds(cubic);
        SkDCubic part = cubic.subDivide(tStart, tEnd);
        SkDQuad quad = part.toQuad();
        if (quad[1].fX < bounds.fLeft) {
            quad[1].fX = bounds.fLeft;
        } else if (quad[1].fX > bounds.fRight) {
            quad[1].fX = bounds.fRight;
        }
        if (quad[1].fY < bounds.fTop) {
            quad[1].fY = bounds.fTop;
        } else if (quad[1].fY > bounds.fBottom) {
            quad[1].fY = bounds.fBottom;
        }
        quads.push_back(quad);
        tStart = tEnd;
    }
}

void CubicPathToQuads(skiatest::Reporter* reporter, const SkPath& cubicPath, SkPath* quadPath) {
    quadPath->reset();
    SkDCubic cubic;
    SkTArray<SkDQuad, true> quads;
    for (auto [verb, pts, w] : SkPathPriv::Iterate(cubicPath)) {
        switch (verb) {
            case SkPathVerb::kMove:
                quadPath->moveTo(pts[0].fX, pts[0].fY);
                continue;
            case SkPathVerb::kLine:
                quadPath->lineTo(pts[1].fX, pts[1].fY);
                break;
            case SkPathVerb::kQuad:
                quadPath->quadTo(pts[1].fX, pts[1].fY, pts[2].fX, pts[2].fY);
                break;
            case SkPathVerb::kCubic:
                quads.reset();
                cubic.set(pts);
                CubicToQuads(cubic, cubic.calcPrecision(), quads);
                for (int index = 0; index < quads.count(); ++index) {
                    SkPoint qPts[2] = {
                        quads[index][1].asSkPoint(),
                        quads[index][2].asSkPoint()
                    };
                    quadPath->quadTo(qPts[0].fX, qPts[0].fY, qPts[1].fX, qPts[1].fY);
                }
                break;
            case SkPathVerb::kClose:
                 quadPath->close();
                break;
            default:
                SkDEBUGFAIL("bad verb");
                return;
        }
    }
	OpDebugOut("start " + reporter->testname + "\n");
	quadPath->dumpHex();
	OpDebugOut("end " + reporter->testname + "\n");
}
#endif

OP_TINY_MAIN(runTests);  // for cmake
