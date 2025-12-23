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
    { run_simplify_quads_tests, "quad", 124032, 30046752 }, // through test 1635740: max pixelError:0.00247338437 testQuads1411840
    { run_simplify_rect_tests, "rects", 152, 1280660 },
    { run_simplify_triangles_tests, "triangle", 24768, 2130048 },
    { run_tiger_tests, "tiger", 7005, 700005 },
};

std::vector<std::string> highError = {
"testQuads1868284", // "0.00301069394"
"testQuads2441981", // "0.00984116644"
"testQuads2449734", // "0.00984228961"
"testQuads2465237", // "0.00986487791"
"testQuads2472989", // "0.00987935439"
"testQuads2558547", // "0.0110246753"
"testQuads2558548", // "0.0111738387"
"testQuads2559908", // "0.0112013761"
"testQuads2566300", // "0.0112189064"
"testQuads2567659", // "0.0112196235"
"testQuads2567660", // "0.0112479953"
"testQuads5109541", // "0.0114621641"
"testQuads5109542", // "0.0121286092"    
};

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string requestedFirst = TEST_FIRST;
std::string testFirst = OP_DEBUG_FAST_TEST || SKIP_TO_V0 ? "" : TEST_FIRST;
bool runOneFile = !OP_DEBUG_FAST_TEST && (!requestedFirst.empty() || SKIP_TO_V0);
bool defeatBreak = TEST_DEFEAT_BREAK || (requestedFirst.empty() && !SKIP_TO_V0);
std::string skipToFile = !OP_DEBUG_FAST_TEST && SKIP_TO_V0  ? "v0" : SKIP_TO_FILE;
std::string largestPixelError;
std::atomic_int testIndex = 0; 
bool showTestName = OP_SHOW_TEST_NAME;
std::atomic_int testsRun = 0;
std::atomic_int testsDot = 0;
std::atomic_int testsLine = 0;
std::atomic_int totalRun = 0;
std::atomic_int testsSkipped = 0;
std::atomic_int testsToSkip = 0;
std::atomic_int totalSkipped = 0;
std::atomic_int silentError = 0;
std::atomic_int totalError = 0;
std::atomic_int treeError = 0;
std::atomic_int gapError = 0;
std::atomic<float> pixelError = 0.f;
float maxPixelError = 0.00247338437f;  // testQuads1411840
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

static void showStats() {
    std::string s;
    if (!currentTestFile.empty())
        s += currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped);
    if (currentTestFile.empty() || testsRun != totalRun || testsSkipped != totalSkipped)
        s += "  total run:" + STR(totalRun) + " skipped:" + STR(totalSkipped);
    if (totalError)
        s += "  error count:" + STR(totalError);
    if (treeError)
        s += "  treeErr count:" + STR(treeError);
    if (gapError)
        s += "  gapErr count:" + STR(gapError);
    if (pixelError && totalRun)
        s += "  avg pixel error:" + STR(pixelError / totalRun);
    if (maxPixelError)
        s += "  max pixel error:" + STR(maxPixelError) + " " + largestPixelError;
    if (!s.empty())
        OpDebugOut(s + "\n");
}

void initTests(std::string filename) {
    totalRun += testsRun;
    totalSkipped += testsSkipped;
    showStats();
    currentTestFile = filename;
    testsRun = 0;
    testsDot = 0;
    testsLine = 0;
    testsSkipped = 0;
    testsToSkip = TESTS_TO_SKIP;
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
    showStats();
}

void CheckForError(const OpDebugData& debugData, bool mayFail) {
    OP_ASSERT(mayFail || debugData.success);
    if (startFirstTest && runOneFile)
        endFirstTest = true;
    if (!debugData.success || mayFail)
        return;
    pixelError += debugData.error; 
#if OP_DEBUG_FAST_TEST
    std::lock_guard<std::mutex> guard(out_mutex);
#else
    if (debugData.error > maxPixelError) {
        OpDebugOut("pixelError:" + STR(debugData.error) + " " + debugData.testname + "\n");
        largestPixelError = debugData.testname;
    }
#endif
    maxPixelError = std::max(debugData.error, maxPixelError);
    const int MAX_ERRORS = 9;
    if (debugData.error <= MAX_ERRORS)
        return;
    totalError++;
    std::string s = debugData.testname;
    if (debugData.error)
        s += " error:" + STR(debugData.error);
    OpDebugOut(s + "\n");
    OpNop();  // for setting a breakpoint
}

#include "port/SkiaPaths.h"
#include "curves/BinaryWinding.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

std::string debugOpTest(std::string testname, const SkPath& pathA, const SkPath& pathB, SkPathOp op) {
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
//    s += "static struct TestDesc tests[] = {\n";
//    s += "    TEST(" + testname + "),\n";
    return s;
}

// char* so it can be called from immediate window
void dumpOpTest(std::string testname, const SkPath& pathA, const SkPath& pathB, SkPathOp op, 
            std::string filename) {
    std::string filePath = dmpFileToPath(filename);
    FILE* file = fopen(filePath.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filePath + " to write\n");
        return;
    }
    std::string s = debugOpTest(testname, pathA, pathB, op);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}

std::string debugSimplifyTest(std::string testname, const SkPath& path) {
	std::string s;
    s += "void " + testname + "(skiatest::Reporter* reporter, const char* filename) {\n";
    s += "    SkPath path;\n";
	s += dumpSkPath(&path, false) + "\n";
	s += "    testSimplify(reporter, path, filename);\n";
    s += "}\n\n";
//    s += "static struct TestDesc tests[] = {\n";
//    s += "    TEST(" + testname + "),\n";
	return s;
}

// char* so it can be called from immediate window
void dumpSimplifyTest(std::string testname, const SkPath& path, std::string filename) {
    std::string filePath = dmpFileToPath(filename);
    FILE* file = fopen(filePath.c_str(), "w");
    if (!file) {
        OpDebugOut("could not open " + filePath + " to write\n");
        return;
    }
	std::string s = debugSimplifyTest(testname, path);
    fwrite(&s[0], 1, s.size(), file);
    fclose(file);
}

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

extern void alt_cubicOp130a();
extern void alt_loop1asQuad();
extern void alt_testArc();
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
    Contour* left = SetSkiaOpContourCallbacks(context, leftData, sizeof(leftData), 
            BinaryOperand::left  OP_DEBUG_PARAMS(&a));
    AddSkiaPath(context, left, a);
    int rightData[] = { 0, 1 };
    Contour* right = SetSkiaOpContourCallbacks(context, rightData, sizeof(rightData), 
            BinaryOperand::right  OP_DEBUG_PARAMS(&b));
    AddSkiaPath(context, right, b);
#if TEST_RASTER
    DebugRaster debugRaster((OpContext*) context);
    if (OpDebugExpect::success == debugDataPtr->expect)
        debugRaster.in();
#endif
    Resolve(context);
    ((OpContext*) context)->dumpFile("OpV0 resolved");
    if (SkPathOpInvertOutput(op, a.isInverseFillType(), b.isInverseFillType()))
        result->toggleInverseFillType();
    ContextError contextError = Error(context);
	trackError(contextError);
#if TEST_RASTER
    if (OpDebugExpect::success == debugDataPtr->expect)
        debugDataPtr->error = debugRaster.out();
#endif
    DeleteContext(context);
	return ContextError::none == contextError;
}

void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail) {
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
	if (alt("cubicOp130a", alt_cubicOp130a))
		return;
	if (alt("loop1asQuad", alt_loop1asQuad))
		return;
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
    SkPath result;
    result.setFillType(SkPathFillType::kEvenOdd);  // !!! workaround
    OpDebugData debugData(testname, v0MayFail ? OpDebugExpect::fail : OpDebugExpect::success,
            CURVE_CURVE_1, CURVE_CURVE_2, CURVE_CURVE_DEPTH, 
            defeatBreak, TEST_DEFEAT_DUMPS, runOneFile
        );
    if (runOneFile)
        dumpOpTest(testname, a, b, op, TestInFile);
	(void) OpV0(a, b, op, &result, &debugData);
    CheckForError(debugData, v0MayFail);
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* name, bool v0MayFail) {
    if (skipTest(name)) {
        return true;
	}
    threadablePathOpTest(0, a, b, op, name, v0MayFail);
    return true;
}

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testname) {
	std::string s(testname);
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    }
    return testPathOpBase(r, a, b, op, testname, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname, bool checkFail) {
    testPathOpBase(r, a, b, op, testname, false);
}

void testPathOpFuzz(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname) {
	std::string s(testname);
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    }
    testPathOpBase(r, a, b, op, testname, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    return testPathOpBase(r, a, b, op, testName, false);
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
    debugRaster.deleteOld();
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
        ((OpContext*) context)->dumpFile("SimplifyV0 resolved");
		contextError = Error(context);
		if (ContextError::toVertical == contextError)
			veryLarge = VeryLargeSkiaPath(path);			
		trackError(contextError);
	}
#if TEST_RASTER
    if (OpDebugExpect::success == optional->expect)
        optional->error = debugRaster.out();
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
            SkPath& out, bool v0MayFail) {
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
    OpDebugData debugData(testname, v0MayFail ? OpDebugExpect::fail : OpDebugExpect::success, 
            CURVE_CURVE_1, CURVE_CURVE_2, CURVE_CURVE_DEPTH, 
            defeatBreak, TEST_DEFEAT_DUMPS, runOneFile
        );
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
    if (runOneFile)
        dumpSimplifyTest(testname, path, TestInFile);
#endif
	(void) SimplifyV0(path, &out, &debugData);
    CheckForError(debugData, v0MayFail);
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
    threadableSimplifyTest(0, path, testname.c_str(), out, false);
    return true;
}

bool testSimplifyBase(skiatest::Reporter* r, const SkPath& path, const char* name, 
        bool v0MayFail) {
    if (skipTest(name)) {
        return true;
	}
    SkPath out;
    threadableSimplifyTest(0, path, name, out, v0MayFail);
    return true;
}

bool testSimplify(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, false);
}

bool testSimplifyFail(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, true);
}

bool testSimplifyFuzz(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    return testSimplifyBase(r, path, testname, true);
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

OP_TINY_MAIN(runTests);  // for cmake
