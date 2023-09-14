/*
 * Copyright 2012 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkPath.h"
#include "include/core/SkRegion.h"
//#include "include/core/SkString.h"
#include "include/pathops/SkPathOps.h"
//#include "include/private/SkMacros.h"
#include "OpDebug.h"
#include "PathOps.h"

namespace skiatest {
    struct Reporter {
        bool allowExtendedTest() { return true; }
        bool verbose() { return false; }
    };

    struct PathOpsThreadState {
        PathOpsThreadState(int a, int b, int c, int d)
            : fA(a), fB(b), fC(c), fD(d) {
        }
        unsigned char fA;
        unsigned char fB;
        unsigned char fC;
        unsigned char fD;
        std::string fPathStr;
        const char* fKey;
        char fSerialNo[256];
        skiatest::Reporter* fReporter;
        SkBitmap* fBitmap;

        void outputProgress(const char* pathStr, SkPathFillType) {}
        void outputProgress(const char* pathStr, SkPathOp) {}
    };

}

#define DEF_TEST(name, reporter)                                \
    static void test_##name(skiatest::Reporter*);               \
    void test_##name(skiatest::Reporter* reporter)

static void initializeTests(skiatest::Reporter* , const char* ) {
}

struct PathOpsThreadedTestRunner {
    PathOpsThreadedTestRunner(skiatest::Reporter*) {}
    void render() {}
};

using namespace skiatest;

// !!! move to Skia test utilities, I guess
const int bitWidth = 64;
const int bitHeight = 64;

static void debug_scale_matrix(const SkPath& one, SkMatrix& scale) {
    SkRect larger = one.getBounds();
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

void VerifySimplify(const SkPath& one, const SkPath& result) {
    SkPath pathOut, scaledPathOut;
    SkRegion rgnA, openClip, rgnOut;
    openClip.setRect({ -16000, -16000, 16000, 16000 });
    rgnA.setPath(one, openClip);
    rgnA.getBoundaryPath(&pathOut);
    SkMatrix scale;
    debug_scale_matrix(one, scale);
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

// four rects, of four sizes
// for 3 smaller sizes, tall, wide
    // top upper mid lower bottom aligned (3 bits, 5 values)
    // same with x (3 bits, 5 values)
// not included, square, tall, wide (2 bits)
// cw or ccw (1 bit)

static void testSimplify4x4RectsMain(PathOpsThreadState* data)
{
    SkASSERT(data);
    PathOpsThreadState& state = *data;
    int aShape = state.fA & 0x03;
    SkPathDirection aCW = state.fA >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
    int bShape = state.fB & 0x03;
    SkPathDirection bCW = state.fB >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
    int cShape = state.fC & 0x03;
    SkPathDirection cCW = state.fC >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
    int dShape = state.fD & 0x03;
    SkPathDirection dCW = state.fD >> 2 ? SkPathDirection::kCCW : SkPathDirection::kCW;
    for (int aXAlign = 0; aXAlign < 5; ++aXAlign) {
        for (int aYAlign = 0; aYAlign < 5; ++aYAlign) {
            for (int bXAlign = 0; bXAlign < 5; ++bXAlign) {
                for (int bYAlign = 0; bYAlign < 5; ++bYAlign) {
                    for (int cXAlign = 0; cXAlign < 5; ++cXAlign) {
                         for (int cYAlign = 0; cYAlign < 5; ++cYAlign) {
                            for (int dXAlign = 0; dXAlign < 5; ++dXAlign) {
    for (int dYAlign = 0; dYAlign < 5; ++dYAlign) {
        SkString pathStr;
        SkPath path, out;
        int l SK_INIT_TO_AVOID_WARNING, t SK_INIT_TO_AVOID_WARNING,
            r SK_INIT_TO_AVOID_WARNING, b SK_INIT_TO_AVOID_WARNING;
        if (aShape) {
            switch (aShape) {
                case 1:  // square
                    l =  0; r = 60;
                    t =  0; b = 60;
                    aXAlign = 5;
                    aYAlign = 5;
                    break;
                case 2:
                    l =  aXAlign * 12;
                    r =  l + 30;
                    t =  0; b = 60;
                    aYAlign = 5;
                    break;
                case 3:
                    l =  0; r = 60;
                    t =  aYAlign * 12;
                    b =  l + 30;
                    aXAlign = 5;
                    break;
            }
            path.addRect(SkIntToScalar(l), SkIntToScalar(t), SkIntToScalar(r), SkIntToScalar(b),
                    aCW);
            if (state.fReporter->verbose()) {
                pathStr.appendf("    path.addRect(%d, %d, %d, %d,"
                        " SkPathDirection::kC%sW);\n", l, t, r, b,
                                aCW == SkPathDirection::kCCW ? "C" : "");
            }
        } else {
            aXAlign = 5;
            aYAlign = 5;
        }
        if (bShape) {
            switch (bShape) {
                case 1:  // square
                    l =  bXAlign * 10;
                    r =  l + 20;
                    t =  bYAlign * 10;
                    b =  l + 20;
                    break;
                case 2:
                    l =  bXAlign * 10;
                    r =  l + 20;
                    t =  10; b = 40;
                    bYAlign = 5;
                    break;
                case 3:
                    l =  10; r = 40;
                    t =  bYAlign * 10;
                    b =  l + 20;
                    bXAlign = 5;
                    break;
            }
            path.addRect(SkIntToScalar(l), SkIntToScalar(t), SkIntToScalar(r), SkIntToScalar(b),
                    bCW);
            if (state.fReporter->verbose()) {
                pathStr.appendf("    path.addRect(%d, %d, %d, %d,"
                        " SkPathDirection::kC%sW);\n", l, t, r, b,
                                bCW == SkPathDirection::kCCW ? "C" : "");
            }
        } else {
            bXAlign = 5;
            bYAlign = 5;
        }
        if (cShape) {
            switch (cShape) {
                case 1:  // square
                    l =  cXAlign * 6;
                    r =  l + 12;
                    t =  cYAlign * 6;
                    b =  l + 12;
                    break;
                case 2:
                    l =  cXAlign * 6;
                    r =  l + 12;
                    t =  20; b = 30;
                    cYAlign = 5;
                    break;
                case 3:
                    l =  20; r = 30;
                    t =  cYAlign * 6;
                    b =  l + 20;
                    cXAlign = 5;
                    break;
            }
            path.addRect(SkIntToScalar(l), SkIntToScalar(t), SkIntToScalar(r), SkIntToScalar(b),
                    cCW);
            if (state.fReporter->verbose()) {
                pathStr.appendf("    path.addRect(%d, %d, %d, %d,"
                        " SkPathDirection::kC%sW);\n", l, t, r, b,
                                cCW == SkPathDirection::kCCW ? "C" : "");
            }
        } else {
            cXAlign = 5;
            cYAlign = 5;
        }
        if (dShape) {
            switch (dShape) {
                case 1:  // square
                    l =  dXAlign * 4;
                    r =  l + 9;
                    t =  dYAlign * 4;
                    b =  l + 9;
                    break;
                case 2:
                    l =  dXAlign * 6;
                    r =  l + 9;
                    t =  32; b = 36;
                    dYAlign = 5;
                    break;
                case 3:
                    l =  32; r = 36;
                    t =  dYAlign * 6;
                    b =  l + 9;
                    dXAlign = 5;
                    break;
            }
            path.addRect(SkIntToScalar(l), SkIntToScalar(t), SkIntToScalar(r), SkIntToScalar(b),
                    dCW);
            if (state.fReporter->verbose()) {
                pathStr.appendf("    path.addRect(%d, %d, %d, %d,"
                        " SkPathDirection::kC%sW);\n", l, t, r, b,
                                dCW == SkPathDirection::kCCW ? "C" : "");
            }
        } else {
            dXAlign = 5;
            dYAlign = 5;
        }
        path.close();
        if (state.fReporter->verbose()) {
            state.outputProgress(pathStr.c_str(), SkPathFillType::kWinding);
        }
        testSimplify(path, false, out, state, pathStr.c_str());
        if (state.fReporter->verbose()) {
            state.outputProgress(pathStr.c_str(), SkPathFillType::kEvenOdd);
        }
        testSimplify(path, true, out, state, pathStr.c_str());
    }
                            }
                        }
                    }
                }
            }
        }
    }
}

DEF_TEST(PathOpsSimplifyRectsThreaded, reporter) {
    initializeTests(reporter, "testLine");
    PathOpsThreadedTestRunner testRunner(reporter);
    for (int a = 0; a < 8; ++a) {  // outermost
        for (int b = a ; b < 8; ++b) {
            for (int c = b ; c < 8; ++c) {
                for (int d = c; d < 8; ++d) {
                    PathOpsThreadState s(a, b, c, d);
                    testSimplify4x4RectsMain(&s);
//                    *testRunner.fRunnables.append() = new PathOpsThreadedRunnable(
//                            &testSimplify4x4RectsMain, a, b, c, d, &testRunner);
                }
                if (!reporter->allowExtendedTest()) goto finish;
            }
        }
    }
finish:
    testRunner.render();
}

void testFail(SkPath& p, SkPathFillType fillType = SkPathFillType::kWinding) {
    SkPath out;
    skiatest::PathOpsThreadState unused(0, 0, 0, 0);
    testSimplify(p, SkPathFillType::kEvenOdd == fillType, out, unused, "");
}

void fail1() {
    SkPath path;
path.moveTo(0, 0);
path.lineTo(20, 0);
path.lineTo(20, 20);
path.lineTo(0, 20);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(12, 0);
path.lineTo(12, 12);
path.lineTo(0, 12);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(9, 0);
path.lineTo(9, 9);
path.lineTo(0, 9);
path.lineTo(0, 0);
path.close();
testFail(path);
}

void fail2() {
    SkPath path;
path.moveTo(0, 0);
path.lineTo(20, 0);
path.lineTo(20, 20);
path.lineTo(0, 20);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(12, 0);
path.lineTo(12, 12);
path.lineTo(0, 12);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 16);
path.lineTo(9, 16);
path.lineTo(9, 9);
path.lineTo(0, 9);
path.lineTo(0, 16);
path.close();
testFail(path);
}

void fail3() {
    SkPath path;
path.moveTo(0, 0);
path.lineTo(20, 0);
path.lineTo(20, 20);
path.lineTo(0, 20);
path.lineTo(0, 0);
path.close();
path.moveTo(6, 6);
path.lineTo(18, 6);
path.lineTo(18, 18);
path.lineTo(6, 18);
path.lineTo(6, 6);
path.close();
path.moveTo(12, 4);
path.lineTo(21, 4);
path.lineTo(21, 21);
path.lineTo(12, 21);
path.lineTo(12, 4);
path.close();
testFail(path, SkPathFillType::kEvenOdd);
}

void fail4() {
    SkPath path;
path.moveTo(0, 10);
path.lineTo(20, 10);
path.lineTo(20, 20);
path.lineTo(0, 20);
path.lineTo(0, 10);
path.close();
path.moveTo(0, 0);
path.lineTo(12, 0);
path.lineTo(12, 12);
path.lineTo(0, 12);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 12);
path.lineTo(9, 12);
path.lineTo(9, 9);
path.lineTo(0, 9);
path.lineTo(0, 12);
path.close();
testFail(path);
}

void fail5() {
    SkPath path;
path.moveTo(10, 20);
path.lineTo(30, 20);
path.lineTo(30, 30);
path.lineTo(10, 30);
path.lineTo(10, 20);
path.close();
path.moveTo(6, 24);
path.lineTo(18, 24);
path.lineTo(18, 18);
path.lineTo(6, 18);
path.lineTo(6, 24);
path.close();
path.moveTo(12, 0);
path.lineTo(21, 0);
path.lineTo(21, 21);
path.lineTo(12, 21);
path.lineTo(12, 0);
path.close();
testFail(path);
}

void fail6() {
    SkPath path;
path.moveTo(12, 0);
path.lineTo(42, 0);
path.lineTo(42, 60);
path.lineTo(12, 60);
path.lineTo(12, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 20);
path.lineTo(20, 20);
path.lineTo(20, 0);
path.lineTo(0, 0);
path.close();
path.moveTo(6, 0);
path.lineTo(6, 18);
path.lineTo(18, 18);
path.lineTo(18, 0);
path.lineTo(6, 0);
path.close();
path.moveTo(4, 0);
path.lineTo(4, 13);
path.lineTo(13, 13);
path.lineTo(13, 0);
path.lineTo(4, 0);
path.close();
testFail(path);
}

void run_all_tests2() {
    fail6();
    fail5();
    fail4();
    fail3();
    fail2();
    fail1();
    skiatest::Reporter* reporter = nullptr;
    test_PathOpsSimplifyRectsThreaded(reporter);
}
