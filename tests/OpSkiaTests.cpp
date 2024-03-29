// (c) 2023, Cary Clark cclark2@gmail.com
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
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
#if OP_BULK_THREADS
#include <mutex>
#include <thread>
#else
#include "CTPL_old/ctpl_stl.h"
#endif
#endif
#include <atomic>
#include <vector>

struct testInfo {
    void (*func)();
    std::string name;
    int count;  // approximate number is ok; used for thread partitioning
    int extended; 
};

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
    { run_op_rect_tests, "rect", 157392, 3194640 },
    { run_simplify_tests, "simplify", 465, 465 },
    { run_simplify_degenerate_tests, "simplifyDegenerate", 47872, 2345984 },
    { run_simplify_fail_tests, "simplifyFail", 7, 7 },
    { run_simplify_quadralaterals_tests, "simplifyQuadralaterals", 124032, 30046752 },
    { run_simplify_quads_tests, "simplifyQuads", 124032, 30046752 },
    { run_simplify_rect_tests, "simplifyRect", 152, 1280660 },
    { run_simplify_triangles_tests, "simplifyTriangles", 24768, 2130048 },
    { run_tiger_tests, "tiger", 7005, 700005 },
};

// skip tests by filename
std::vector<std::string> skipTestFiles = { TEST_PATH_OP_SKIP_FILES };
std::vector<std::string> skipRestFiles = { TEST_PATH_OP_SKIP_REST };
std::string testFirst = OP_DEBUG_FAST_TEST ? "" : TEST_PATH_OP_FIRST;
std::string skipToFile = TEST_PATH_OP_SKIP_TO_FILE;
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
#if OP_BULK_THREADS
thread_local std::string currentTestFile;
thread_local int firstSuiteTest = 0;
thread_local int lastSuiteTest = 0;
#if OP_DEBUG_FAST_TEST
std::mutex out_mutex;
#endif
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

#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_BULK_THREADS
ctpl::thread_pool threadpool(OP_MAX_THREADS);
#endif

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
    OpCurve c = { { OpDebugBitsToFloat(0x42f16017), OpDebugBitsToFloat(0x431b7908) }, // {120.688, 155.473}
        { OpDebugBitsToFloat(0x42ed5d28), OpDebugBitsToFloat(0x43205806) } }; // {118.682, 160.344}
    OpCurve rotated = c.toVertical(line);
    OP_ASSERT(rotated.pts[0].x == OpDebugBitsToFloat(0x390713e0)     // 0.00012882007
            || rotated.pts[0].x == OpDebugBitsToFloat(0x39000000));  // 0.00012207031
    runWithFMA = rotated.pts[0].x == OpDebugBitsToFloat(0x390713e0);
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
    if ("" != testFirst)
        startFirstTest = true;
#if OP_DEBUG_FAST_TEST && OP_BULK_THREADS
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

void bulkTest(int index) {
    int totalTests = 0;
    int suiteCount = 0;
    for (auto testSuite : testSuites) {
        if (skipToFile.size() && testSuite.name != skipToFile) {
            if (!totalTests)
                ++suiteCount;
            continue;
        }
        totalTests += OP_TEST_ALLOW_EXTENDED ? testSuite.extended : testSuite.count;
    }
    int firstTest = index * totalTests / OP_MAX_THREADS;
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
            currentTestFile = testSuites[suiteCount].name;
            (testSuites[suiteCount].func)();
            if (lastTest <= 0)
                return;
            firstTest = 0;
        }
        ++suiteCount;
    }
}

uint64_t timerFrequency;
uint64_t timerStart;

void runTests() {
#if !OP_DEBUG_FAST_TEST || !OP_BULK_THREADS
    auto runTest = [](std::string s) {
        for (auto suite : testSuites) {
            if (suite.name == s) {
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
#if !OP_SHOW_ERRORS_ONLY
                OpDebugOut(suite.name + " started\n");
#endif
                currentTestFile = suite.name;
#else
                initTests(suite.name);
#endif
                (*suite.func)();
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_SHOW_ERRORS_ONLY
                OpDebugOut(suite.name + " finished\n");
#endif
#if OP_SHOW_TEST_COUNT
                OpDebugOut("," + STR(testsSkipped) + "\n");
                testsSkipped = 0;
#endif
                return;
            }
        }
    };
	OpDebugOut("\n");
#endif
    timerFrequency = OpInitTimer();
    timerStart = OpReadTimer();
#if OP_DEBUG_FAST_TEST && OP_BULK_THREADS
    std::vector<std::thread> t;
    for (int index = 0; index < OP_MAX_THREADS; ++index)
        t.push_back(std::thread(bulkTest, index));
    for (int index = 0; index < OP_MAX_THREADS; ++index)
        t[index].join();
#else
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
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_BULK_THREADS
    threadpool.stop(true);
#endif
    uint64_t end = OpReadTimer();
    float elapsed = OpTicksToSeconds(end - timerStart, timerFrequency);
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS
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

void threadablePathOpTest(int id, const SkPath& a, const SkPath& b, 
        SkPathOp op, std::string testname, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
    SkPath result;
	OpInPath op1(&a);
	OpInPath op2(&b);
	OpOutPath opOut(&result);
    std::vector<OpDebugWarning> warnings;
#if OP_DEBUG || OP_TEST_REGION
    bool success = 
#endif
#if OP_DEBUG
        DebugPathOps(op1, op2, (OpOperator) op, opOut, v0MayFail ? OpDebugExpect::unknown :
                OpDebugExpect::success, testname, warnings);
#else
        PathOps(op1, op2, (OpOperator) op, opOut);
#endif
    OP_ASSERT(success || v0MayFail);
#endif
#if OP_TEST_SKIA
    SkPath skresult;
    bool skSuccess = Op(a, b, op, &skresult);
    OP_ASSERT(skSuccess || skiaMayFail);
#if OP_TEST_V0
    if (success && !skSuccess)
        testsPassSkiaFail++;
    else if (!success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#elif OP_TEST_REGION
    bool skSuccess = true;
#endif
    if (startFirstTest && "" != testFirst)
        endFirstTest = true;
#if OP_TEST_V0 && OP_TEST_REGION
    if (!success || !skSuccess || v0MayFail || skiaMayFail)
        return;
    int errors = VerifyOp(a, b, op, testname, result);
//  int altErrors = VerifyOpNoRegion(a, b, op, result);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS || warnings.size()) {
#if !defined(NDEBUG) || OP_RELEASE_TEST
        ReportError(testname, errors, warnings);
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
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_BULK_THREADS
    std::string testname(name);
    a.updateBoundsCache();
    b.updateBoundsCache();
    threadpool.push(threadablePathOpTest, a, b, op, testname, v0MayFail, skiaMayFail);
#else
    threadablePathOpTest(0, a, b, op, name, v0MayFail, skiaMayFail);
#endif
    return true;
}

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testname) {
    std::string s = std::string(testname);
    std::vector<std::string> skip = { TEST_PATH_OP_EXCEPTIONS };  // see OpTestDrive.h
    if (skip.end() != std::find(skip.begin(), skip.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> lap = { LAPTOP_PATH_OP_EXCEPTIONS };  // see OpTestDrive.h
    if (!runningWithFMA() && lap.end() != std::find(lap.begin(), lap.end(), s) && s != testFirst) {
        ++testsSkipped;
        return true;
    }
    std::vector<std::string> fuzz = { TEST_PATH_OP_MAP_TO_FUZZ };  // see OpTestDrive.h
    if (fuzz.end() != std::find(fuzz.begin(), fuzz.end(), s) && s != testFirst)
        return (void) testPathOpFuzz(r, a, b, op, testname), true;
    std::vector<std::string> lapz = { LAPTOP_PATH_OP_MAP_TO_FUZZ };  // see OpTestDrive.h
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
    testPathOpBase(r, a, b, op, testname, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    std::vector<std::string> fail = { TEST_PATH_OP_FAIL_EXCEPTIONS };  // see OpTestDrive.h
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

void threadableSimplifyTest(int id, const SkPath& path, std::string testname, 
            SkPath& out, bool v0MayFail, bool skiaMayFail) {
#if OP_TEST_V0
	OpInPath op1(&path);
    out.reset();
	OpOutPath opOut(&out);
    std::vector<OpDebugWarning> warnings;
#if OP_DEBUG || OP_TEST_REGION
    bool success =
#endif
#if OP_DEBUG
        DebugPathSimplify(op1, opOut, v0MayFail ? OpDebugExpect::unknown :
                OpDebugExpect::success, testname, warnings);
#else
        PathSimplify(op1, opOut);
#endif
    OP_ASSERT(v0MayFail || success);
#endif
#if OP_TEST_SKIA
    SkPath skOut;
    bool skSuccess = Simplify(path, &skOut);
    OP_ASSERT(skiaMayFail || skSuccess);
#if OP_TEST_V0
    if (success && !skSuccess)
        testsPassSkiaFail++;
    else if (!success && skSuccess)
        testsFailSkiaPass++;
#else
    if (skiaMayFail && skSuccess)
        testsFailSkiaPass++;
#endif
#endif
    if (startFirstTest && "" != testFirst)
        endFirstTest = true;
#if OP_TEST_V0 && OP_TEST_REGION
    if (!success)
        return;
    int errors = VerifySimplify(path, testname, out);
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
    #if !defined(NDEBUG) || OP_RELEASE_TEST
        ReportError(testname, errors, warnings);
        testsError++;
    }
    #endif
#endif
}

bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& state, 
        const char* name) {
    std::string testname(name);
    if ("" == testname) {
        testname = state.fReporter->testname + STR(++state.fReporter->unnamedCount);
    }
    if (skipTest(testname))
        return true;
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_BULK_THREADS
    path.updateBoundsCache();
    threadpool.push(threadableSimplifyTest, path, testname, out, false, false);
#else
    threadableSimplifyTest(0, path, testname.c_str(), out, false, false);
#endif
    return true;
}

bool testSimplifyBase(skiatest::Reporter* r, const SkPath& path, const char* name, 
        bool v0MayFail, bool skiaMayFail) {
    if (skipTest(name))
        return true;
    SkPath out;
#if OP_DEBUG_FAST_TEST && OP_TEST_ENABLE_THREADS && !OP_BULK_THREADS
    std::string testname(name);
    path.updateBoundsCache();
    threadpool.push(threadableSimplifyTest, path, testname, out, v0MayFail, skiaMayFail);
#else
    threadableSimplifyTest(0, path, name, out, v0MayFail, skiaMayFail);
#endif
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
