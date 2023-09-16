/*
 * Copyright 2015 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpDebugSkiaTests.h"

using namespace skiatest;

#include <atomic>

static int loopNo = 4;
static std::atomic<int> gCirclesTestNo{0};

static void testOpCirclesMain(PathOpsThreadState* data) {
        SkASSERT(data);
    const SkPathFillType fts[] = { SkPathFillType::kWinding, SkPathFillType::kEvenOdd };
    PathOpsThreadState& state = *data;
    SkString pathStr;
    for (int a = 0 ; a < 6; ++a) {
        for (int b = a + 1 ; b < 7; ++b) {
            for (int c = 0 ; c < 6; ++c) {
                for (int d = c + 1 ; d < 7; ++d) {
                    for (auto e : fts) {
    for (auto f : fts) {
        SkPath pathA, pathB;
        pathA.setFillType(e);
        pathA.addCircle(SkIntToScalar(state.fA), SkIntToScalar(state.fB), SkIntToScalar(state.fC),
                state.fD ? SkPathDirection::kCW : SkPathDirection::kCCW);
        pathB.setFillType(f);
        pathB.addCircle(SkIntToScalar(a), SkIntToScalar(b), SkIntToScalar(c),
                d ? SkPathDirection::kCW : SkPathDirection::kCCW);
        for (int op = 0 ; op <= kXOR_SkPathOp; ++op)    {
            if (state.fReporter->verbose()) {
                pathStr.printf("static void circlesOp%d(skiatest::Reporter* reporter,"
                        " const char* filename) {\n", loopNo);
                pathStr.appendf("    SkPath path, pathB;\n");
                pathStr.appendf("    path.setFillType(SkPathFillType::k%s);\n",
                        e == SkPathFillType::kWinding ? "Winding" : e == SkPathFillType::kEvenOdd
                        ? "EvenOdd" : "?UNDEFINED");
                pathStr.appendf("    path.addCircle(%d, %d, %d, %s);\n", state.fA, state.fB,
                        state.fC, state.fD ? "SkPathDirection::kCW" : "SkPathDirection::kCCW");
                pathStr.appendf("    pathB.setFillType(SkPathFillType::k%s);\n",
                        f == SkPathFillType::kWinding ? "Winding" : f == SkPathFillType::kEvenOdd
                        ? "EvenOdd" : "?UNDEFINED");
                pathStr.appendf("    pathB.addCircle(%d, %d, %d, %s);\n", a, b,
                        c, d ? "SkPathDirection::kCW" : "SkPathDirection::kCCW");
                pathStr.appendf("    testPathOp(reporter, path, pathB, %s, filename);\n",
                        SkPathOpsDebug::OpStr((SkPathOp) op));
                pathStr.appendf("}\n");
                state.outputProgress(pathStr.c_str(), (SkPathOp) op);
            }
            SkString testName;
            testName.printf("thread_circles%d", ++gCirclesTestNo);
            if (!testPathOp(state.fReporter, pathA, pathB, (SkPathOp) op, testName.c_str())) {
                if (state.fReporter->verbose()) {
                    ++loopNo;
                    goto skipToNext;
                }
            }
            if (PathOpsDebug::gCheckForDuplicateNames) return;
        }
    }
                    }
skipToNext: ;
                }
            }
        }
    }
}

DEF_TEST(PathOpsOpCircleThreaded, reporter) {
    initializeTests(reporter, "circleOp");
    PathOpsThreadedTestRunner testRunner(reporter);
    for (int a = 0; a < 6; ++a) {  // outermost
        for (int b = a + 1; b < 7; ++b) {
            for (int c = 0 ; c < 6; ++c) {
                for (int d = 0; d < 2; ++d) {
                    PathOpsThreadState s(a, b, c, d);
                    testOpCirclesMain(&s);
//                    *testRunner.fRunnables.append() = new PathOpsThreadedRunnable(
//                            &testOpCirclesMain, a, b, c, d, &testRunner);
                }
            }
            if (!reporter->allowExtendedTest()) goto finish;
        }
    }
finish:
    testRunner.render();
}

void testFail(SkPath& a, SkPath& b, SkPathOp op) {
    skiatest::Reporter* reporter = nullptr;
    SkPath out;
    skiatest::PathOpsThreadState unused(0, 0, 0, 0);
    testPathOp(reporter, a, b, op, "");
}

void thread_circles14545() {
SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 1);
path.conicTo(1, 0, 0, 0, 0.707106769f);
path.conicTo(-1, 0, -1, 1, 0.707106769f);
path.conicTo(-1, 2, 0, 2, 0.707106769f);
path.conicTo(1, 2, 1, 1, 0.707106769f);
path.close();
SkPath a(path);
path.reset();
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 2);
path.conicTo(1, 3, 0, 3, 0.707106769f);
path.conicTo(-1, 3, -1, 2, 0.707106769f);
path.conicTo(-1, 1, 0, 1, 0.707106769f);
path.conicTo(1, 1, 1, 2, 0.707106769f);
path.close();
SkPath b(path);
testFail(a, b, kDifference_SkPathOp);
}

void run_all_circle_tests() {
    thread_circles14545();
    skiatest::Reporter* reporter = nullptr;
    test_PathOpsOpCircleThreaded(reporter);
}
