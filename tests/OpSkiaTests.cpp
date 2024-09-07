// (c) 2023, Cary Clark cclark2@gmail.com

// !!! test this ! (OpCurve::rawIntersect commented out code)
// for thread_circles104483, edges 113 and 117 fail to find intersection; check for error here

// switches that decide which tests to run and how to run them
// these may be moved to command line parameters at some point
#define TESTS_TO_SKIP 0 // 14295903  // tests to skip
#define TEST_PATH_OP_FIRST "loop2"  // e.g., "cubic140721" (ignored by fast test)

#define OP_SHOW_TEST_NAME 0  // if 0, show a dot every 100 tests
#define OP_SHOW_ERRORS_ONLY 0  // if 1, skip showing dots, test files started/finished
#define OP_TEST_ALLOW_EXTENDED 1  // some Skia tests have extended versions which take longer
                                  // (out of date) max run: 8,430,493: skipped: 5 error: 335
#define OP_TEST_V0 1  // set to zero to time Skia running tests
#define OP_TEST_SKIA 1  // see if skia's path ops can execute the same test
#define OP_TEST_REGION 1  // test result of v0 against skia regions

#define CURVE_CURVE_1 6  // id of segment 1 to break in divide and conquer
#define CURVE_CURVE_2 10  // id of segment 2 to break in divide and conquer
#define CURVE_CURVE_DEPTH 5  // minimum recursion depth for curve curve break (-1 to disable)

// see descriptions for exceptions below
#define TEST_PATH_OP_EXCEPTIONS "issue3517" // "cubics7d"
#define TEST_PATH_OP_FUZZ_EXCEPTIONS "fuzz487a", "fuzz487b" // "fuzz487a"
#define TEST_PATH_OP_FAIL_EXCEPTIONS "grshapearcs1" // "grshapearcs1"
#define TEST_PATH_OP_MAP_TO_FUZZ "fuzzhang_1" // "fuzzhang_1"
#define TEST_PATH_SIMPLIFY_EXCEPTIONS "" // 
#define TEST_PATH_SIMPLIFY_FAIL_EXCEPTIONS "grshapearc" // "grshapearc"
#define TEST_PATH_SIMPLIFY_FUZZ_EXCEPTIONS ""
#define TEST_PATH_SIMPLIFY_MAP_TO_FUZZ  ""

// !!! need to update laptop exceptions with latest
#define LAPTOP_PATH_OP_EXCEPTIONS ""
#define LAPTOP_PATH_OP_MAP_TO_FUZZ ""
#define LAPTOP_SIMPLIFY_EXCEPTIONS "joel_5"

// when these tests are encountered, it and the remaining tests in the file are skipped
#define TEST_PATH_OP_SKIP_REST ""
#define TEST_PATH_OP_SKIP_FILES ""  /* e.g., "battle", "circleOp" */

/* test failure descriptions:
extended: 25213840 / 30046752 quad tests run 8/01/24

extended: 30046752 (all) quadralateral tests run

testQuadralaterals21723521 had errors=20

(old)

fuzz763_378: no edge found: last, last resort (x2) had errors=36 # this looks fixable
             gap between edge 2225 and 2227 (why is 2226 disabled?)
             likely other errors

(old)
  last successful run desktop:
total run:735269 skipped:0 errors:1 warnings:26 v0 only:3 skia only:70
  last successful run laptop:
total run:735268 skipped:1 errors:2 warnings:41 v0 only:4 skia only:70

issue3517 no edge found: last, last resort
fuzzhang_1: succeeds in skia, fails in v0 (investigate)

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

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string testFirst = OP_DEBUG_FAST_TEST || TEST_PATH_OP_SKIP_TO_V0 
        ? "" : TEST_PATH_OP_FIRST;
bool runOneFile = (strlen(TEST_PATH_OP_FIRST) || TEST_PATH_OP_SKIP_TO_V0) && !OP_DEBUG_FAST_TEST;
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
std::atomic_int testsError;
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
    totalError += testsError;
    totalWarn += testsWarn;
    totalFailSkiaPass += testsFailSkiaPass;
    totalPassSkiaFail += testsPassSkiaFail;
    if (testsRun || testsSkipped || totalRun || totalSkipped)
        OpDebugOut(currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped)
                + " err:" + STR(testsError) + " warn:" + STR(testsWarn)
                + " v0:" + STR(testsPassSkiaFail) + " sk:" + STR(testsFailSkiaPass)
                + " total run:" + STR(totalRun) + " skipped:" + STR(totalSkipped)
                + " err:" + STR(totalError) + " warn:" + STR(totalWarn)
                + " v0:" + STR(totalPassSkiaFail) + " sk:" + STR(totalFailSkiaPass) + "\n");
    currentTestFile = filename;
    testsRun = 0;
    testsDot = 0;
    testsLine = 0;
    testsSkipped = 0;
    testsError = 0;
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
            + " errors:" + STR(testsError) +  " warnings:" + STR(testsWarn) 
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

static int debug_paths_draw_the_same(const SkPath& one, const SkPath& two, SkBitmap& bits) {
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
    OP_ASSERT(errors < 9);
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
        const SkPath& result) {
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
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap);
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

void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail) {
    const char* tn = testname.c_str();
#if OP_TEST_V0
    SkPath result;
    result.setFillType(SkPathFillType::kEvenOdd);  // !!! workaround
    OpDebugData debugData(v0MayFail);
    debugData.debugTestname = testname;
    debugData.debugCurveCurve1 = CURVE_CURVE_1;
    debugData.debugCurveCurve2 = CURVE_CURVE_2;
    debugData.debugCurveCurveDepth = CURVE_CURVE_DEPTH;
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
        DeleteContext(context);
    }
#endif
#if OP_TEST_SKIA
    SkPath skresult;
    bool skSuccess = Op(a, b, op, &skresult);
    OP_ASSERT(skSuccess || skiaMayFail);
#if OP_TEST_V0
    if (debugData.debugSuccess && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.debugSuccess && skSuccess)
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
    if (!debugData.debugSuccess || !skSuccess || v0MayFail || skiaMayFail)
        return;
    int errors = VerifyOp(a, b, op, testname, result);
//  int altErrors = VerifyOpNoRegion(a, b, op, result);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS || debugData.debugWarnings.size()) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
#endif
        dumpOpTest(tn, a, b, op)
            ;   // <<<<<<<< paste this into immediate window
        ReportError(testname, errors, debugData.debugWarnings);
        if (errors > MAX_ERRORS)
            testsError++;
#endif
    }

#endif
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* name, bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name))
        return true;
    threadablePathOpTest(0, a, b, op, name, v0MayFail, skiaMayFail);
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
    return testPathOpBase(r, a, b, op, testname, false, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testname, bool checkFail) {
    testPathOpBase(r, a, b, op, testname, false, false);
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
    testPathOpBase(r, a, b, op, testname, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    std::vector<std::string> fail = { TEST_PATH_OP_FAIL_EXCEPTIONS };
    if (fail.end() != std::find(fail.begin(), fail.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    return testPathOpBase(r, a, b, op, testName, false, true);
}

void RunTestSet(skiatest::Reporter* r, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter* , const char* testName),
        void (*skipTest)(skiatest::Reporter* , const char* testName),
        void (*stopTest)(skiatest::Reporter* , const char* testName), bool reverse) {
    for (size_t i = 0; i < count; ++i)
        (*tests[i].fun)(r, tests[i].str);
}


int VerifySimplify(const SkPath& one, std::string testname, const SkPath& result) {
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
    int errors = debug_paths_draw_the_same(scaledPathOut, scaledOut, bitmap);
    return errors;
}

// char* so it can be called from immediate window
void dumpSimplifyTest(const char* testname, const SkPath& path) {
    OpDebugOut("\nvoid ");
    OpDebugOut(testname);
    OpDebugOut("(skiatest::Reporter* reporter, const char* filename) {\n");
    OpDebugOut("    SkPath path;\n");
    path.dump();
    OpDebugOut("    testSimplify(reporter, path, filename);\n");
    OpDebugOut("}\n\n");
    OpDebugOut("static struct TestDesc tests[] = {\n");
    OpDebugOut("    TEST(" + std::string(testname) + "),\n");
}

void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
    out.setFillType(SkPathFillType::kEvenOdd); // !!! workaround
    std::vector<OpDebugWarning> warnings;
    const char* tn = testname.c_str();
    if ("never!" == testname)
        OpDebugOut(tn);  // prevent optimizer from removing tn
    OpDebugData debugData(v0MayFail);
    debugData.debugTestname = testname;
    debugData.debugCurveCurve1 = CURVE_CURVE_1;
    debugData.debugCurveCurve2 = CURVE_CURVE_2;
    debugData.debugCurveCurveDepth = CURVE_CURVE_DEPTH;
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
        DeleteContext(context);
    }
    OP_ASSERT(v0MayFail || debugData.debugSuccess);
#endif
#if OP_TEST_SKIA
    SkPath skOut;
    bool skSuccess = Simplify(path, &skOut);
    OP_ASSERT(skiaMayFail || skSuccess);
#if OP_TEST_V0
    if (debugData.debugSuccess && !skSuccess)
        testsPassSkiaFail++;
    else if (!debugData.debugSuccess && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#endif
    if (startFirstTest && runOneFile)
        endFirstTest = true;
#if OP_TEST_V0 && OP_TEST_REGION
    if (!debugData.debugSuccess)
        return;
    int errors = VerifySimplify(path, testname, out);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
#if OP_DEBUG_FAST_TEST
        std::lock_guard<std::mutex> guard(out_mutex);
#endif
        dumpSimplifyTest(testname.c_str(), path)
            ;   // <<<<<<<< paste this into immediate window
        ReportError(testname, errors, warnings);
#endif
        testsError++;
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
