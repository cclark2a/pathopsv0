// (c) 2023, Cary Clark cclark2@gmail.com
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkRegion.h"
#include "OpDebugSkiaTests.h"
#include "PathOps.h"

// skip tests by filename
std::vector<std::string> skipTestFiles = {"circleOp"};
// execute specific named test first
std::string currentTestFile;
std::string testFirst; // = "thread_circles7489";
bool skiatest::Reporter::allowExtendedTest() { return false; }

bool showTestName = false;
int testsRun = 0;
int totalRun = 0;
int testsSkipped = 0;
int totalSkipped = 0;

bool PathOpsDebug::gCheckForDuplicateNames = false;
bool PathOpsDebug::gJson = false;

void initializeTests(skiatest::Reporter* , const char* filename) {
    if (currentTestFile.size()) {
        totalRun += testsRun;
        totalSkipped += testsSkipped;
        OpDebugOut(currentTestFile + " run:" + STR(testsRun) + " skipped:" + STR(testsSkipped)
                + " total:" + STR(totalRun) + " skipped:" + STR(totalSkipped) + "\n");
    }
    currentTestFile = filename;
    testsRun = 0;
    testsSkipped = 0;
    OpDebugOut(currentTestFile + "\n");
}

bool skipTest(skiatest::Reporter* r, const char* testname) {
    std::string name = std::string(testname);
    if ("" != testFirst && name != testFirst) {
        ++testsSkipped;
        return true;
    }
    if (r) {
        std::string filename = r->filename + '_' + r->subname;
        if (currentTestFile != filename)
            initializeTests(r, filename.c_str());
    }
    if (skipTestFiles.end() != std::find(skipTestFiles.begin(), skipTestFiles.end(), 
            currentTestFile)) {
        ++testsSkipped;
        return true;
    }
    ++testsRun;
    if (showTestName)
        OpDebugOut(name + "\n");
    else if (testsRun % 100 == 0) {
        OpDebugOut(".");
        if (testsRun % 5000 == 0)
            OpDebugOut("\n");
    }
    return false;
}

// !!! move to Skia test utilities, I guess
const int bitWidth = 64;
const int bitHeight = 64;

static void debug_scale_matrix(const SkPath& one, const SkPath* two, SkMatrix& scale) {
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

void VerifyOp(const SkPath& one, const SkPath& two, SkPathOp op,
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
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
        fprintf(stderr, "// Op did not expect errors=%d\n", errors);
//        DumpOp(stderr, one, two, op, "opTest");
        fflush(stderr);
        OP_ASSERT(0);
    }
}

bool testPathOpBase(skiatest::Reporter* r, const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* testname, bool v0MayFail, bool skiaMayFail) {
    if (skipTest(r, testname))
        return true;
    SkPath result, skresult, xorResult;
	OpInPath op1(&a);
	OpInPath op2(&b);
	OpOutPath opOut(&result);
    bool success = 
#if OP_DEBUG
        DebugPathOps(op1, op2, (OpOperator) op, opOut, v0MayFail ? OpDebugExpect::unknown :
                OpDebugExpect::success);
#else
        PathOps(op1, op2, (OpOperator) op, opOut);
#endif
    OP_ASSERT(success || v0MayFail);
    bool skSuccess = Op(a, b, op, &skresult);
    OP_ASSERT(skSuccess || skiaMayFail);
    if ((0) && testname == std::string("op_1")) {
        OpDebugOut("v0 result:\n");
        result.dump();
        OpDebugOut("sk result:\n");
        skresult.dump();
        OpDebugOut("");
    }
#if 0
    bool xorSucess = Op(result, skresult, kXOR_SkPathOp, &xorResult);
    OP_ASSERT(xorSucess);
    OP_ASSERT(xorResult.isEmpty());
#else
    if (success && skSuccess && !v0MayFail && !skiaMayFail) VerifyOp(a, b, op, result);
#endif
    return true;
}

bool testPathOp(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    if (s == "pentrek4")  // complicated input; worth debugging at some point
        return true;
    if (s == "issue3517")  // long and skinny; don't know what's going on
        return true;       // unterminated ends of contour are edges 1434, 1441
    if (s == "thread_circles7489")  // pair of conics do not find intersection
        return true;       // when reduced to line/conic, there's error finding roots
    if (s == "fuzzhang_1") // test passes in skia, not in v0; but its a fuzz, don't care for now
        return (void) testPathOpFuzz(r, a, b, op, testName), true;
    return testPathOpBase(r, a, b, op, testName, false, false);
}

void testPathOpCheck(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testName, bool checkFail) {
    testPathOpBase(r, a, b, op, testName, false, false);
}

void testPathOpFuzz(skiatest::Reporter* r, const SkPath& a, const SkPath& b, SkPathOp op, 
        const char* testName) {
    testPathOpBase(r, a, b, op, testName, true, true);
}

bool testPathOpFail(skiatest::Reporter* r, const SkPath& a, const SkPath& b,
        const SkPathOp op, const char* testName) {
    std::string s = std::string(testName);
    if (s == "grshapearcs1") // very complicated; defer. Asserts in OpWinder::AddLineCurveIntersection
        return true;        // a 'more code needed?' alert that oSegment pinned to bounds
    return testPathOpBase(r, a, b, op, testName, false, true);
}

void RunTestSet(skiatest::Reporter* r, TestDesc tests[], size_t count,
        void (*firstTest)(skiatest::Reporter* , const char* testName),
        void (*skipTest)(skiatest::Reporter* , const char* testName),
        void (*stopTest)(skiatest::Reporter* , const char* testName), bool reverse) {
    for (size_t i = 0; i < count; ++i)
        (*tests[i].fun)(r, tests[i].str);
}

void VerifySimplify(const SkPath& one, const SkPath& result) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, openClip, rgnOut;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnA.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, nullptr, scale);
    SkRegion scaledRgnA, scaledRgnB, scaledRgnOut;
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
    const int MAX_ERRORS = 9;
    if (errors > MAX_ERRORS) {
        fprintf(stderr, "// Op did not expect errors=%d\n", errors);
        fflush(stderr);
        OP_ASSERT(0);
    }
}

bool testSimplify(SkPath& path, bool useXor, SkPath& out, PathOpsThreadState& , 
        const char* testname) {
    if (skipTest(nullptr, testname))
        return true;
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
	OpInPath op1(&path);
    out.reset();
	OpOutPath opOut(&out);
    bool success = PathSimplify(op1, opOut);
    OP_ASSERT(success);
    SkPath skOut;
    OP_DEBUG_CODE(bool skSuccess =) Simplify(path, &skOut);
    OP_ASSERT(skSuccess);
    if (success) 
        VerifySimplify(path, out);
    return true;
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
    runner->fRunnables.data = { a, b, c, d };
}

PathOpsThreadedTestRunner::PathOpsThreadedTestRunner(skiatest::Reporter*) {
    fRunnables.slot = nullptr;
}

void PathOpsThreadedTestRunner::render() {
    fRunnables.append();
}
