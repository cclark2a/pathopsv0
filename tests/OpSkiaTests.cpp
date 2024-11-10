// (c) 2023, Cary Clark cclark2@gmail.com

// !!! test this ! (OpCurve::rawIntersect commented out code)
// for thread_circles104483, edges 113 and 117 fail to find intersection; check for error here

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TEST_PATH_OP_SKIP_TO_FILE "v0" // e.g., "quad" tests only (see testSuites in OpSkiaTests)
#define TESTS_TO_SKIP 0 // 14295903  // tests to skip
#define TEST_FIRST ""  // e.g., "joel4" (ignored by fast test, overridden by TEST_DRIVE_FIRST)
// fuzz763_378 asserts in OpIntersections::sort() debug check line 397 but continuing, succeeds
// grshapearc hangs in OpTree contructor? (makes over 10K limbs)
#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_ALLOW_EXTENDED 1  // some Skia tests have extended versions which take longer
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions

#define CURVE_CURVE_1 7  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 2  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH -1  // minimum recursion depth for curve curve break (-1 to disable)

// see descriptions for exceptions below
#define TEST_PATH_OP_EXCEPTIONS "" // "pentrek10"
#define TEST_PATH_OP_FUZZ_EXCEPTIONS "" // "fuzz487a", "fuzz487b"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "" // "grshapearcs1"
#define TEST_PATH_OP_MAP_TO_FUZZ "" // "fuzzhang_1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "" // 
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "" // "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  ""

// !!! need to update laptop exceptions with latest
#define LAPTOP_PATH_OP_EXCEPTIONS ""
#define LAPTOP_PATH_OP_MAP_TO_FUZZ ""
#define LAPTOP_SIMPLIFY_EXCEPTIONS "" //  "joel_5"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

/* test failure descriptions:
extended: all tests run 11/9/24 exceptions: grshapearc (total run:74600014 v0 only:13)
*/

#if !OP_TINY_SKIA
const void* dummyLeft;
const void* dummyRight;
#endif


#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkRegion.h"
#endif
#include "OpSkiaTests.h"
#include "OpContour.h"
#include "OpCurve.h"
#if OP_DEBUG_FAST_TEST
  #include <mutex>
  #include <thread>
  #define OP_MAX_THREADS std::thread::hardware_concurrency()
#else
 #define OP_MAX_THREADS 1
#endif
#include <atomic>
#include <vector>

struct testInfo {
    void (*func)();
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
#if !OP_TINY_SKIA
    { run_op_tests, "op", 451, 451 },
#endif
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

std::vector<std::string> fails = {
    "fuzz767834",
    "fuzz754434_1",
    "fuzz754434_2",
    "fuzz754434_3",
    "fuzz754434_4",
    "fuzzhang_3",
    "fuzzhang_2",
    "fuzzhang_1",
    "fuzz763_57",
    "fuzz763_56",
    "fuzz763_55",
    "fuzz763_54",
    "fuzz763_53",
    "fuzz763_51",
    "fuzz763_50",
    "fuzz763_49",
    "fuzz763_48",
    "fuzz763_45",
    "fuzz763_43",
    "fuzz763_42",
    "fuzz763_41",
    "fuzz763_40",
    "fuzz763_39",
    "fuzz763_38",
    "fuzz763_37",
    "fuzz763_35",
    "fuzz763_34",
    "fuzz763_33",
    "fuzz763_32",
    "fuzz763_31",
    "fuzz763_30",
    "fuzz763_29",
    "fuzz763_28",
    "fuzz763_26",
    "fuzz763_25",
    "fuzz763_24",
    "fuzz763_23",
    "fuzz763_22",
    "fuzz763_21",
    "fuzz763_20",
    "fuzz763_19",
    "fuzz763_18",
    "fuzz763_17",
    "fuzz763_16",
    "fuzz763_14",
    "fuzz763_13",
    "fuzz763_12",
    "fuzz763_11",
    "fuzz763_10",
    "kfuzz2",
    "fuzz763_7",
    "fuzz763_6",
    "fuzz763_3a",
    "fuzz763_1b",
    "fuzz763_9",
    "fuzz714",
    "fuzz487a",
    "fuzz487b",
    "op_1",
    "op_2",
    "op_3",
    "fuzz_k1",
    "fuzz_x3",
    "fuzz763_2s",
    "fuzz763_1",
    "grshapearc"
};

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string requestedFirst = strlen(TEST_DRIVE_FIRST) ? TEST_DRIVE_FIRST : TEST_FIRST;
std::string testFirst = OP_DEBUG_FAST_TEST || TEST_PATH_OP_SKIP_TO_V0 
        ? "" : requestedFirst;
bool runOneFile = (!requestedFirst.empty() || TEST_PATH_OP_SKIP_TO_V0) && !OP_DEBUG_FAST_TEST;
std::string skipToFile = TEST_PATH_OP_SKIP_TO_V0 && !OP_DEBUG_FAST_TEST 
        ? "v0" : TEST_PATH_OP_SKIP_TO_FILE;
std::atomic_int testIndex; 
bool showTestName = OP_SHOW_TEST_NAME;
std::atomic_int testsRun;
std::atomic_int testsDot;
std::atomic_int testsLine;
std::atomic_int totalRun;
std::atomic_int testsSkipped;
std::atomic_int totalSkipped;
std::atomic_int silentError;
std::atomic_int totalError;
extern std::atomic_int testsWarn;
std::atomic_int totalWarn;
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

// short-circuit extended if only one test is run
bool skiatest::Reporter::allowExtendedTest() {
    return "" == testFirst ? OP_TEST_ALLOW_EXTENDED : !endFirstTest;
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
    totalWarn += testsWarn;
    totalFailSkiaPass += testsFailSkiaPass;
    totalPassSkiaFail += testsPassSkiaFail;
    if (testsRun || testsSkipped || totalRun || totalSkipped)
        OpDebugOut(currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped)
                + " warn:" + STR(testsWarn)
                + " v0:" + STR(testsPassSkiaFail) + " sk:" + STR(testsFailSkiaPass)
                + " total run:" + STR(totalRun) + " skipped:" + STR(totalSkipped)
                + " err:" + STR(totalError) + " warn:" + STR(totalWarn)
                + " v0:" + STR(totalPassSkiaFail) + " sk:" + STR(totalFailSkiaPass) + "\n");
    currentTestFile = filename;
    testsRun = 0;
    testsDot = 0;
    testsLine = 0;
    testsSkipped = 0;
    testsWarn = 0;
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
                && testsDot > (OP_TEST_ALLOW_EXTENDED ? 5000 : 500)) {
            OpDebugOut(".");
            testsDot -= OP_TEST_ALLOW_EXTENDED ? 5000 : 500;
            if (testsLine > (OP_TEST_ALLOW_EXTENDED ? 500000 : 50000)) {
                OpDebugOut("\n");
                testsLine -= OP_TEST_ALLOW_EXTENDED ? 500000 : 50000;
            }
        }
    }
    return false;
}

// if skipToFile is set, run a single test divided among threads
void bulkTest(int index) {
    int totalTests = 0;
    for (auto testSuite : testSuites) {
        if (skipToFile.size() && testSuite.name != skipToFile)
            continue;
        totalTests += OP_TEST_ALLOW_EXTENDED ? testSuite.extended : testSuite.count;
    }
    int firstTest = index * totalTests / OP_MAX_THREADS;
    firstTest += TESTS_TO_SKIP / OP_MAX_THREADS;
    int lastTest = (index + 1) * totalTests / OP_MAX_THREADS;
    for (auto testSuite : testSuites) {
        if (skipToFile.size() && testSuite.name != skipToFile)
            continue;
        int testCount = OP_TEST_ALLOW_EXTENDED ? testSuite.extended : testSuite.count;
        firstTest -= testCount;
        lastTest -= testCount;
        if (firstTest < 0) {
            firstSuiteTest = testCount + firstTest;
            lastSuiteTest = testCount + lastTest;
            currentTestFile = testSuite.name;
            needsName = testSuite.extended != testSuite.count;
            unnamedCount = 0;
            (testSuite.func)();
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
    auto runTest = [](std::string s) {
        for (auto suite : testSuites) {
            if (suite.name == s) {
                currentTestFile = suite.name;
                needsName = suite.extended != suite.count;
                unnamedCount = 0;
                initTests(suite.name);
                (*suite.func)();
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
            + " errors:" + STR(totalError) +  " warnings:" + STR(testsWarn) 
            + " v0 only:" + STR(testsPassSkiaFail) + " skia only:" + STR(testsFailSkiaPass) + "\n");
}

const int bitWidth = 64;
const int bitHeight = 64;

static SkRect debug_scale_matrix(const SkPath& one, const SkPath* two, SkMatrix& scale) {
    SkRect larger = one.getBounds();
    if (two) {
        larger.join(two->getBounds());
    }
    SkScalar largerWidth = larger.width();
    if (largerWidth < 4) {
        largerWidth = 4;
    }
    SkScalar largerHeight = larger.height();
    if (largerHeight < 4) {
        largerHeight = 4;
    }
    SkScalar hScale = (bitWidth - 2) / largerWidth;
    SkScalar vScale = (bitHeight - 2) / largerHeight;
    scale.reset();
    scale.preScale(hScale, vScale);
    larger.fLeft *= hScale;
    larger.fRight *= hScale;
    larger.fTop *= vScale;
    larger.fBottom *= vScale;
    SkScalar dx = -16000 > larger.fLeft ? -16000 - larger.fLeft
        : 16000 < larger.fRight ? 16000 - larger.fRight : 0;
    SkScalar dy = -16000 > larger.fTop ? -16000 - larger.fTop
        : 16000 < larger.fBottom ? 16000 - larger.fBottom : 0;
    scale.preTranslate(dx, dy);
    return larger;
}

static int debug_paths_draw_the_same(const SkPath& one, const SkPath& two, SkBitmap& bits,
        bool v0mayFail, bool assertOnError) {
    if (bits.width() == 0) {
        bits.allocN32Pixels(bitWidth * 2, bitHeight);
    }
    SkCanvas canvas(bits);
    canvas.drawColor(SK_ColorWHITE);
    SkPaint paint;
    canvas.save();
    const SkRect& bounds1 = one.getBounds();
    canvas.translate(-bounds1.fLeft + 1, -bounds1.fTop + 1);
    canvas.drawPath(one, paint);
    canvas.restore();
    canvas.save();
    canvas.translate(-bounds1.fLeft + 1 + bitWidth, -bounds1.fTop + 1);
    canvas.drawPath(two, paint);
    canvas.restore();
    int errors = 0;
    for (int y = 0; y < bitHeight - 1; ++y) {
        uint32_t* addr1 = bits.getAddr32(0, y);
        uint32_t* addr2 = bits.getAddr32(0, y + 1);
        uint32_t* addr3 = bits.getAddr32(bitWidth, y);
        uint32_t* addr4 = bits.getAddr32(bitWidth, y + 1);
        for (int x = 0; x < bitWidth - 1; ++x) {
            // count 2x2 blocks
            bool err = addr1[x] != addr3[x];
            if (err) {
                errors += addr1[x + 1] != addr3[x + 1]
                    && addr2[x] != addr4[x] && addr2[x + 1] != addr4[x + 1];
            }
        }
    }
    OP_ASSERT(!assertOnError || errors < 9 || v0mayFail);
    return errors;
}

std::vector<std::string> warningStrings = { "no edge found: last, last resort" };

void ReportError(std::string testname, int errors, std::vector<OpDebugWarning>& warnings) {
    std::string s = testname;
    // !!! when there's more than one warning, put this in a loop or lambda or something
    int count = 0;
    OpDebugWarning test = OpDebugWarning::lastResort;
    for (OpDebugWarning w : warnings) {
        if (test == w)
            ++count;
    }
    if (count) {
        s += " " + warningStrings[(int) test];
        if (count > 1)
            s += " (x" + STR(count) + ")";
    }
    if (errors)
        s += " had errors=" + STR(errors);
    OpDebugOut(s + "\n");
    OpDebugOut("");  // for setting a breakpoint
}

int VerifyOpNoRegion(const SkPath& left, const SkPath& right, SkPathOp op, const SkPath& result) {
    SkMatrix scale;
    SkRect bounds = debug_scale_matrix(left, &right, scale);
    SkPath scaledLeft, scaledRight, scaledResult;
    scaledLeft.addPath(left, scale);
    scaledLeft.setFillType(left.getFillType());
    scaledRight.addPath(right, scale);
    scaledRight.setFillType(right.getFillType());
    scaledResult.addPath(result, scale);
    scaledResult.setFillType(result.getFillType());
    SkBitmap bitmap;
    bitmap.allocN32Pixels(bitWidth, bitHeight);
    SkCanvas canvas(bitmap);
    canvas.drawColor(SK_ColorBLACK);
    SkPaint paint;
    paint.setBlendMode(SkBlendMode::kPlus);
    constexpr uint32_t leftColor = SK_ColorRED;
    constexpr uint32_t rightColor = SK_ColorBLUE;
    constexpr uint32_t resultColor = SK_ColorGREEN;
    canvas.translate(-bounds.fLeft, -bounds.fTop);
    paint.setColor(leftColor);
    canvas.drawPath(scaledLeft, paint);
    paint.setColor(rightColor);
    canvas.drawPath(scaledRight, paint);
    paint.setColor(resultColor);
    canvas.drawPath(scaledResult, paint);
    std::vector<uint32_t> okColors = { SK_ColorBLACK };
    switch (op) {
        case kDifference_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor);
            okColors.push_back(leftColor | rightColor);
        break;
        case kIntersect_SkPathOp:
            okColors.push_back(leftColor);
            okColors.push_back(rightColor);
            okColors.push_back(leftColor | rightColor | resultColor);
        break;
        case kUnion_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor | resultColor);
        break;
        case kXOR_SkPathOp:
            okColors.push_back(leftColor | resultColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor);
        break;
        case kReverseDifference_SkPathOp:
            okColors.push_back(leftColor);
            okColors.push_back(rightColor | resultColor);
            okColors.push_back(leftColor | rightColor);
        break;
        default:
            OP_ASSERT(0);
    }
    int errors = 0;
    for (int y = 0; y < bitHeight - 1; ++y) {
        uint32_t* addr1 = bitmap.getAddr32(0, y);
        for (int x = 0; x < bitWidth - 1; ++x) {
            if (okColors.end() != std::find(okColors.begin(), okColors.end(), addr1[x]))
                continue;
            ++errors;
        }
    }
    if (errors > 9)
        OpDebugOut("");
    return errors;
}

int VerifyOp(const SkPath& one, const SkPath& two, SkPathOp op, std::string testname,
        const SkPath& result, bool v0mayFail) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, rgnB, openClip, rgnOut;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnB.setPath(two, openClip);
    rgnOut.op(rgnA, rgnB, (SkRegion::Op)op);
    rgnOut.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, &two, scale);
    SkRegion scaledRgnA, scaledRgnB, scaledRgnOut;
    SkPath scaledA, scaledB;
    scaledA.addPath(one, scale);
    scaledA.setFillType(one.getFillType());
    scaledB.addPath(two, scale);
    scaledB.setFillType(two.getFillType());
    scaledRgnA.setPath(scaledA, openClip);
    scaledRgnB.setPath(scaledB, openClip);
    scaledRgnOut.op(scaledRgnA, scaledRgnB, (SkRegion::Op)op);
    scaledRgnOut.getBoundaryPath(&scaledPathOut);
    SkBitmap bitmap;
    SkPath scaledOut;
    scaledOut.addPath(result, scale);
    scaledOut.setFillType(result.getFillType());
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap, v0mayFail, true);
    return errors;
}

#include "skia/SkiaPaths.h"
#include "curves/BinaryWinding.h"

// char* so it can be called from immediate window
void dumpOpTest(const char* testname, const SkPath& pathA, const SkPath& pathB, SkPathOp op) {
    OpDebugOut("\nvoid ");
    OpDebugOut(testname);
    OpDebugOut("(skiatest::Reporter* reporter, const char* filename) {\n");
    OpDebugOut("    SkPath pathA, path;\n");
    pathA.dump();
    OpDebugOut("    pathA = path;\n");
    OpDebugOut("    path.reset();\n");
    pathB.dump();
    std::string opStr;
    switch(op) {
        case SkPathOp::kDifference_SkPathOp: opStr = "SkPathOp::kDifference_SkPathOp"; break;
        case SkPathOp::kIntersect_SkPathOp: opStr = "SkPathOp::kIntersect_SkPathOp"; break;
        case SkPathOp::kUnion_SkPathOp: opStr = "SkPathOp::kUnion_SkPathOp"; break;
        case SkPathOp::kXOR_SkPathOp: opStr = "SkPathOp::kXOR_SkPathOp"; break;
        case SkPathOp::kReverseDifference_SkPathOp: opStr = "SkPathOp::kReverseDifference_SkPathOp"; break;
        default: OP_ASSERT(0);
    }
    OpDebugOut("    testPathOp(reporter, pathA, path, " + opStr + ", filename);\n");
    OpDebugOut("}\n\n");
    OpDebugOut("static struct TestDesc tests[] = {\n");
    OpDebugOut("    TEST(" + std::string(testname) + "),\n");
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

			break;
		default:
			OP_ASSERT(0);
	}
}

// mayDiffer is true if test is fuzz with large values that Skia ignores
void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail, bool mayDiffer) {
    const char* tn = testname.c_str();
#if OP_TEST_V0
    SkPath result;
    result.setFillType(SkPathFillType::kEvenOdd);  // !!! workaround
    OpDebugData debugData(v0MayFail);
    debugData.testname = testname;
    debugData.curveCurve1 = CURVE_CURVE_1;
    debugData.curveCurve2 = CURVE_CURVE_2;
    debugData.curveCurveDepth = CURVE_CURVE_DEPTH;
    {
        using namespace PathOpsV0Lib;
        Context* context = CreateContext({ nullptr, 0 });
        SetSkiaContextCallBacks(context);
        OP_DEBUG_CODE(Debug(context, debugData));
        SetSkiaCurveCallBacks(context);
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
        Contour* left = SetSkiaOpCallBacks(context, mappedOp, BinaryOperand::left, windType
                OP_DEBUG_PARAMS(a));
        int leftData[] = { 1, 0 };
        PathOpsV0Lib::AddWinding leftWinding { left, leftData, sizeof(leftData) };
        AddSkiaPath(leftWinding, a);
        Contour* right = SetSkiaOpCallBacks(context, mappedOp, BinaryOperand::right, windType
                OP_DEBUG_PARAMS(b));
        int rightData[] = { 0, 1 };
        PathOpsV0Lib::AddWinding rightWinding { right, rightData, sizeof(rightData) };
        AddSkiaPath(rightWinding, b);
        PathOutput pathOutput = &result;
        Resolve(context, pathOutput);
        if (SkPathOpInvertOutput(op, a.isInverseFillType(), b.isInverseFillType()))
            result.toggleInverseFillType();
		ContextError contextError = Error(context);
		trackError(contextError);
        DeleteContext(context);
    }
#endif
#if OP_TEST_SKIA
    SkPath skresult;
    bool skSuccess = Op(a, b, op, &skresult);
    OP_ASSERT(skSuccess || skiaMayFail);
#if OP_TEST_V0
    if (debugData.success && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#elif OP_TEST_REGION
    bool skSuccess = true;
#endif
    if (startFirstTest && runOneFile)
        endFirstTest = true;
#if OP_TEST_V0 && OP_TEST_REGION
    if (!debugData.success || !skSuccess || v0MayFail || skiaMayFail || mayDiffer)
        return;
    int errors = VerifyOp(a, b, op, testname, result, v0MayFail);
//  int altErrors = VerifyOpNoRegion(a, b, op, result);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS || debugData.warnings.size()) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
#endif
        dumpOpTest(tn, a, b, op)
            ;   // <<<<<<<< paste this into immediate window
        ReportError(testname, errors, debugData.warnings);
        if (errors > MAX_ERRORS)
            totalError++;
#endif
    }

#endif
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* name, bool v0MayFail, bool skiaMayFail, bool mayDiffer) {
    if (skipTest(name))
        return true;
    threadablePathOpTest(0, a, b, op, name, v0MayFail, skiaMayFail, mayDiffer);
    return true;
}

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testname) {
    std::string s;
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    } else
        s = std::string(testname);
    std::vector<std::string> skip = { TEST_PATH_OP_EXCEPTIONS };
    if (skip.end() != std::find(skip.begin(), skip.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> lap = { LAPTOP_PATH_OP_EXCEPTIONS };
    if (!runningWithFMA() && lap.end() != std::find(lap.begin(), lap.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> fuzz = { TEST_PATH_OP_MAP_TO_FUZZ };
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst)
        return (void) testPathOpFuzz(r, a, b, op, testname), true;
    std::vector<std::string> lapz = { LAPTOP_PATH_OP_MAP_TO_FUZZ };
    if (!runningWithFMA() && lapz.end() != std::find(lapz.begin(), lapz.end(), s) && s != testFirst)
        return (void) testPathOpFuzz(r, a, b, op, testname), true;
    return testPathOpBase(r, a, b, op, testname, false, false, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname, bool checkFail) {
    testPathOpBase(r, a, b, op, testname, false, false, true);
}

void testPathOpFuzz(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname) {
    std::string s;
    if (needsName) {
        s = currentTestFile + STR(++unnamedCount);
        testname = s.c_str();
    } else
        s = std::string(testname);
    std::vector<std::string> skip = { TEST_PATH_OP_FUZZ_EXCEPTIONS };
    if (skip.end() != std::find(skip.begin(), skip.end(), s) && s != testFirst) {
        ++testsSkipped;
        return;
    }
    testPathOpBase(r, a, b, op, testname, true, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    std::vector<std::string> fail = { TEST_PATH_OP_FAIL_EXCEPTIONS };
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testPathOpBase(r, a, b, op, testName, false, true, true);
}

void RunTestSet(skiatest::Reporter* r, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter* , const char* testName),
        void (*skipTest)(skiatest::Reporter* , const char* testName),
        void (*stopTest)(skiatest::Reporter* , const char* testName), bool reverse) {
    for (size_t i = 0; i < count; ++i)
        (*tests[i].fun)(r, tests[i].str);
}


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

std::string debugSimplifyTest(const char* testname, const SkPath& path) {
	std::string s;
    s += "void " + STR(testname) + "(skiatest::Reporter* reporter, const char* filename) {\n";
    s += "    SkPath path;\n";
#if !OP_TINY_SKIA  // !!! missing dump path equivalent in tiny skia implementation
	s += dumpSkPath(&path, false) + "\n";
#endif
	s += "    testSimplify(reporter, path, filename);\n";
    s += "}\n\n";
    s += "static struct TestDesc tests[] = {\n";
    s += "    TEST(" + std::string(testname) + "),";
	return s;
}

// char* so it can be called from immediate window
void dumpSimplifyTest(const char* testname, const SkPath& path) {
	std::string s = "\n";
	s += debugSimplifyTest(testname, path) + "\n";
    OpDebugOut(s);
}

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
	long allocSize = fileSize + replace.size();
	if (fseek(readf.file, 0 , SEEK_SET))
		return OpDebugOut("fseek to start failed:" + filename + "\n");
 	AutoFree read(malloc(allocSize));
	if (!read.buffer)
		return OpDebugOut("malloc failed:" + readName + "; size:" + STR(allocSize) + "\n");
	size_t bytesRead = fread(read.buffer, 1, fileSize, readf.file);
	if (bytesRead < (size_t) fileSize)
		return OpDebugOut("read failed:" + readName + "; read:" + STR(bytesRead) 
				+ " expected:" + STR(fileSize) + "\n");
	fclose(readf.file);
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
				+ " expected:" + STR(startSize) + "\n:");
	bytesWritten = fwrite(replace.c_str(), 1, replace.size(), write.file);
	if (bytesWritten != replace.size())
		return OpDebugOut("write replace failed:" + filename + "; written:" + STR(bytesWritten)
				+ " expected:" + STR(replace.size()) + "\n:");
	size_t endPos = startSize + match.size();
	bytesWritten = fwrite(read.buffer + endPos, 1, fileSize - endPos, write.file);
	if (bytesWritten < fileSize - endPos)
		return OpDebugOut("write failed:" + writeName + "; written:" + STR(bytesWritten) 
				+ " expected:" + STR(fileSize - endPos) + "\n");
	fclose(write.file);
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

void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
	const SkPath& p = path;
    out.setFillType(SkPathFillType::kEvenOdd); // !!! workaround
    std::vector<OpDebugWarning> warnings;
    const char* tn = testname.c_str();
    if ("never!" == testname)
        OpDebugOut(tn);  // prevent optimizer from removing tn
    OpDebugData debugData(v0MayFail);
    debugData.testname = testname;
    debugData.curveCurve1 = CURVE_CURVE_1;
    debugData.curveCurve2 = CURVE_CURVE_2;
    debugData.curveCurveDepth = CURVE_CURVE_DEPTH;
    {
        using namespace PathOpsV0Lib;
        Context* context = CreateContext({ nullptr, 0 });
        SetSkiaContextCallBacks(context);
        OP_DEBUG_CODE(Debug(context, debugData));
        SetSkiaCurveCallBacks(context);
        auto isWindingFill = [](const SkPath& path) {
            return SkPathFillType::kWinding == path.getFillType()
                    || SkPathFillType::kInverseWinding == path.getFillType();
        }; 
        Contour* simple = SetSkiaSimplifyCallBacks(context, isWindingFill(path)
                OP_DEBUG_PARAMS(path));
        int simpleData[] = { 1 };
        PathOpsV0Lib::AddWinding simpleWinding { simple, simpleData, sizeof(simpleData) };
        AddSkiaPath(simpleWinding, path);
        out.reset();
        out.setFillType(SkPathFillType::kEvenOdd);
        PathOutput pathOutput = &out;
        Resolve(context, pathOutput);
		ContextError contextError = Error(context);
		trackError(contextError);
        DeleteContext(context);
    }
    OP_ASSERT(v0MayFail || debugData.success);
#endif
#if OP_TEST_SKIA
    SkPath skOut;
    bool skSuccess = Simplify(path, &skOut);
    OP_ASSERT(skiaMayFail || skSuccess);
#if OP_TEST_V0
    if (debugData.success && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#endif
    if (startFirstTest && runOneFile)
        endFirstTest = true;
#if OP_TEST_V0 && OP_TEST_REGION
    if (!debugData.success)
        return;
    int errors = VerifySimplify(path, testname, out, v0MayFail);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS && !v0MayFail) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if 0 && OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
		v0(tn, p);
#endif
        dumpSimplifyTest(tn, p)
            ;   // <<<<<<<< paste this into immediate window
        ReportError(testname, errors, warnings);
#endif
        totalError++;
    }
#endif
}

bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& state, 
        const char* name) {
    std::string testname(name);
    if ("" == testname)
        testname = state.fReporter->testname + STR(++unnamedCount);
    if (skipTest(testname))
        return true;
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
    threadableSimplifyTest(0, path, testname.c_str(), out, false, false);
    return true;
}

bool testSimplifyBase(skiatest::Reporter* r, const SkPath& path, const char* name, 
        bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name))
        return true;
    SkPath out;
    threadableSimplifyTest(0, path, name, out, v0MayFail, skiaMayFail);
    return true;
}

bool testSimplify(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fail = { TEST_PATH_SIMPLIFY_EXCEPTIONS };  // see OpTestDrive.h
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> lap = { LAPTOP_SIMPLIFY_EXCEPTIONS };  // see OpTestDrive.h
    if (!runningWithFMA() && lap.end() != std::find(lap.begin(), lap.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> fuzz = { TEST_PATH_SIMPLIFY_MAP_TO_FUZZ };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst)
        return (void) testSimplifyFuzz(r, path, testname), true;
    return testSimplifyBase(r, path, testname, false, false);
}

bool testSimplifyFail(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fail = { TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS };  // see OpTestDrive.h
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testSimplifyBase(r, path, testname, true, true);
}

bool testSimplifyFuzz(skiatest::Reporter* r, const SkPath& path, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> fuzz = { TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
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
