/*
 * Copyright 2012 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "OpDebugSkiaTests.h"

using namespace skiatest;

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
