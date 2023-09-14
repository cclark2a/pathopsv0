// op debug skia tests
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkRegion.h"
#include "OpDebugSkiaTests.h"
#include "PathOps.h"

bool PathOpsDebug::gCheckForDuplicateNames = false;

using namespace skiatest;

void initializeTests(skiatest::Reporter* , const char* ) {
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

bool testPathOpBase(skiatest::Reporter* , const SkPath& a, const SkPath& b, 
        SkPathOp op, const char* filename, bool v0MayFail, bool skiaMayFail) {
    SkPath result, skresult, xorResult;
    std::string name = std::string(filename);
    OpDebugOut(name + "\n");
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
    if ((0) && name == "op_1") {
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
    if (success && skSuccess) VerifyOp(a, b, op, result);
#endif
    return true;
}

bool testPathOp(skiatest::Reporter*, const SkPath& a, const SkPath& b,
        SkPathOp op, const char* filename) {
    return testPathOpBase(nullptr, a, b, op, filename, false, false);
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

bool testSimplify(SkPath& path, bool useXor, SkPath& out, skiatest::PathOpsThreadState& , const char* ) {
    path.setFillType(useXor ? SkPathFillType::kEvenOdd : SkPathFillType::kWinding);
	OpInPath op1(&path);
    out.reset();
	OpOutPath opOut(&out);
    bool success = PathSimplify(op1, opOut);
    OP_ASSERT(success);
    SkPath skOut;
    bool skSuccess = Simplify(path, &skOut);
    OP_ASSERT(skSuccess);
    if (success) 
        VerifySimplify(path, out);
    ++debugTestsRun;
    if ((debugTestsRun % 100) == 0) {
        OpDebugOut(".");
        if ((debugTestsRun % 5000) == 0) 
            OpDebugOut("\n");
    }
    return true;
}
