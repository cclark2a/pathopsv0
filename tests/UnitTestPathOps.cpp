// (c) 2023, Cary Clark cclark2@gmail.com
//       1         2         3         4         5         6         7         8         9         0
//34567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#include "OpDebug.h"
#include "PathOps.h"

#if OP_RELEASE_TEST

// math test
#include <random>
#include "OpMath.h"
#include "OpContour.h"
#include "src/pathops/SkPathOpsConic.h"
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkPath.h"
#include "include/core/SkRegion.h"
#include "include/pathops/SkPathOps.h"
#include "SkiaTestCommon.h"

#define ASSERT(test) assertReport(test, __FUNCTION__, __LINE__, #test)
#define ASSERT_I(test, outI) assertReport(test, __FUNCTION__, __LINE__, outI, #test)

void assertReport(bool test, const char* func, int lineNo, const char* errorStr) {
	if (!test) {
		std::string s = "error in " + std::string(func) + "() line " + std::to_string(lineNo)
			+ ": " + std::string(errorStr) + "\n";
		OpDebugOut(s);
		fprintf(stderr, "error %s\n", errorStr);
		exit(lineNo);
	}
}

void assertReport(bool test, const char* func, int lineNo, int outI, const char* errorStr) {
	if (!test) {
		std::string s = "error i=" + std::to_string(outI);
		OpDebugOut(s);
		fprintf(stderr, "error %d\n", outI);
		assertReport(test, func, lineNo, errorStr);
	}
}

void OpMathTest() {
	// test AddValidTs : not tested against SkDQuad::AddValidTs because of near equal compares
	OpRoots allGood1( 0, .1f, .5f );
	OpRoots allGood2 ( .7f, .9f, 1 );
	OpRoots allBad1 ( 0 - OpEpsilon, 1 + OpEpsilon, OpInfinity );
	OpRoots allBad2 ( -OpInfinity, OpNaN );
	OpRoots someOfEach1 ( 0, .1f, 1 + OpEpsilon );
	OpRoots someOfEach2 ( .5f, OpInfinity );
	int good = allGood1.keepValidTs().count();
	ASSERT_I(3 == good, good);
	good = allGood2.keepValidTs().count();
	ASSERT_I(3 == good, good);
	int bad = allBad1.keepValidTs().count();
	ASSERT_I(0 == bad, bad);
	bad = allBad2.keepValidTs().count();
	ASSERT_I(0 == bad, bad);
	int mix = someOfEach1.keepValidTs().count();
	ASSERT_I(2 == mix, mix);
	mix = someOfEach2.keepValidTs().count();
	ASSERT_I(1 == mix, mix);

	// test Between
	ASSERT(OpMath::Between(0, 0, 0));
	ASSERT(OpMath::Between(0, 0, 1));
	ASSERT(OpMath::Between(0, 1, 1));
	ASSERT(OpMath::Between(1, 1, 1));
	ASSERT(!OpMath::Between(1, 0, 1));
	ASSERT(!OpMath::Between(0, 1, 0));

	// test cases for root finders
	float p[] = { 0, 1, 2, 1000, 2000, -1000, -2000, 0, 0, 0, 1, 1, 1, .0001f, .0000001f };

	// test CubicRootsReal
	for (unsigned index = 0; index < ARRAY_COUNT(p) - 3; ++index) {
		double tValsD[2];
		int sk_result = SkDCubic::RootsReal(p[0], p[1], p[2], p[3], tValsD);
		OpRoots tVals = OpMath::CubicRootsReal(p[0], p[1], p[2], p[3], MatchEnds::none);
		ASSERT(sk_result == (int) tVals.count());
		for (int r = 0; r < tVals.count(); ++r) {
			ASSERT(tVals.roots[r] == tValsD[r]);
		}
	}

	// test CubicRootsValidT
	for (unsigned index = 0; index < ARRAY_COUNT(p) - 3; ++index) {
		double tValsD[2];
		int sk_result = SkDCubic::RootsValidT(p[0], p[1], p[2], p[3], tValsD);
		OpRoots roots = OpMath::CubicRootsValidT(p[0], p[1], p[2], p[3]);
		ASSERT(sk_result == (int) roots.count());
		for (int r = 0; r < roots.count(); ++r) {
			ASSERT(roots.roots[r] == tValsD[r]);
		}
	}

	// test Interp
	ASSERT(75 == OpMath::Interp(50, 100, .5f));

	// test IsFinite
	ASSERT(OpMath::IsFinite(0));
	ASSERT(!OpMath::IsFinite(OpInfinity));
	ASSERT(!OpMath::IsFinite(OpNaN));

	// test IsNaN
	ASSERT(!OpMath::IsNaN(0));
	ASSERT(!OpMath::IsNaN(OpInfinity));
	ASSERT(OpMath::IsNaN(OpNaN));

	// test QuadRootsReal
	for (unsigned index = 0; index < ARRAY_COUNT(p) - 2; ++index) {
		double tValsD[2];
		int sk_result = SkDQuad::RootsReal(p[0], p[1], p[2], tValsD);
		OpRoots roots = OpMath::QuadRootsReal(p[0], p[1], p[2]);
		ASSERT(sk_result == (int) roots.count());
		for (int r = 0; r < roots.count(); ++r) {
			ASSERT(roots.roots[r] == tValsD[r]);
		}
	}

	// test QuadRootsValidT
	for (unsigned index = 0; index < ARRAY_COUNT(p); ++index) {
		double tValsD[2];
		int sk_result = SkDQuad::RootsValidT(p[0], p[1], p[2], tValsD);
		OpRoots roots = OpMath::QuadRootsValidT(p[0], p[1], p[2]);
		ASSERT(sk_result == (int) roots.count());
		for (int r = 0; r < roots.count(); ++r) {
			ASSERT(roots.roots[r] == tValsD[r]);
		}
	}

	// test error reporting: if enabled, output should show: 
	// error in OpMathTest line xxx: 0
	if (0) ASSERT(0);
}

// limit cubics to random numbers within +/-???, assuming that true for most real-world cubics
enum class CubicTest {
	ptAtT,
	axisRay,
	edgeRay,
	doubleRay
};

enum class SpeedTest {
	off,
	on
};

#if 0 // not worth maintaining for now
void OpCubicErrorTest(CubicTest testType, SpeedTest speedTest) {
	std::mt19937 rng;
	std::normal_distribution<float> xy(0, 1024);
	std::uniform_real_distribution<float> t(0, 1);
	std::uniform_int_distribution<int> zeroOne(0, 1);
	OpCubic worst, largest;
	float worstT = OpNaN, largestT = OpNaN, worstDT = OpNaN;
	OpPoint worstPt, worstDPt, largestPt, largestDPt;
	float largestCept, largestDCept, worstXY;
	float worstError = 1, largestError = 0;
	bool worstX = false, largestX = false;
	auto findWorst = [](float x, float dx) {
		float ax = fabsf(x);
		float adx = fabsf(dx);
		if (ax <= OpEpsilon || adx <= OpEpsilon)
			return 0.f;
		return ax > adx ? ax / adx : adx / ax;
	};
	auto findLargest = [](float x, float dx) {
		float ax = fabsf(x);
		float adx = fabsf(dx);
		if (ax <= OpEpsilon || adx <= OpEpsilon)
			return 0.f;
		return fabsf(ax - adx);
	};
	auto recordWorst = [&](const OpCubic& c, float testT, OpPoint pt, OpPoint dPt, float error,
				bool isX) {
		worstError = error;
		worst = c;
		worstT = testT;
		worstPt = pt;
		worstDPt = dPt;
		worstX = isX;
	};
	auto recordPtAtT = [&](const OpCubic& c, float testT, OpPoint pt, OpPoint dPt, float error,
			bool isX) {
		largestError = error;
		largest = c;
		largestT = testT;
		largestPt = pt;
		largestDPt = dPt;
		largestX = isX;
	};
		auto recordPtAtT2 = [&](const OpCubic& c, float someXY, float testT, float testDT, 
				OpPoint pt, OpPoint dPt, float error, bool isX) {
		worstError = error;
		worst = c;
		worstXY = someXY;
		worstT = testT;
		worstDT = testDT;
		worstPt = pt;
		worstDPt = dPt;
		worstX = isX;
	};
	auto recordCept = [&](const OpCubic& c, float testT, float cept, float dCept, float error) {
		largestError = error;
		largest = c;
		largestT = testT;
		largestCept = cept;
		largestDCept = dCept;
	};
	auto runAxisTest = [&](const OpCubic& c, float someXy, 
			rootCellar& cepts, rootCellar dCepts, int roots) {
		std::sort(&cepts[0], &cepts[roots]);
		std::sort(&dCepts[0], &dCepts[roots]);
		for (int index = 0; index < roots; index++) {
			float error = fabsf(cepts[index] - dCepts[index]);
			if (largestError < error)
				recordCept(c, someXy, cepts[index], dCepts[index], error);
			if (0 == error)
				continue;
			OpPoint pt = c.ptAtT(cepts[index]);
			OpPoint dPt = c.ptAtT(dCepts[index]);
			error = findLargest(pt.x, dPt.x);
			if (worstError < error)
				recordPtAtT2(c, someXy, cepts[index], dCepts[index], pt, dPt, error, true);
			error = findLargest(pt.y, dPt.y);
			if (worstError < error)
				recordPtAtT2(c, someXy, cepts[index], dCepts[index], pt, dPt, error, false);
		}
	};
	if (CubicTest::ptAtT != testType)
		worstError = 0;
	uint64_t frequency = OpInitTimer();
	uint64_t startTime = OpReadTimer();
	for (int trial = 0; trial < 100000; ++trial) {
		OpCubic cubic;
		// generate a random cubic
		for (int val = 0; val < 8; ++val)
			*(&cubic.pts[0].x + val) = xy(rng);
		OpContours contours;
		OpContour contour(&contours, OpOperand::left);
		OpSegment segment(cubic.pts, cubicType, &contour);
		segment.intersections.emplace_back(OpPtT(cubic.pts[0], 0), &segment, 0
				OP_DEBUG_PARAMS(IntersectMaker::opCubicErrorTest1));
		segment.intersections.emplace_back(OpPtT(cubic.pts[3], 1), &segment, 0
				OP_DEBUG_PARAMS(IntersectMaker::opCubicErrorTest2));
		segment.makeEdges();
		std::normal_distribution<float> xRange(segment.tightBounds.left, segment.tightBounds.right);
		std::normal_distribution<float> yRange(segment.tightBounds.top, segment.tightBounds.bottom);
		if (CubicTest::edgeRay == testType) {
			for (int test = 0; test < 100; ++test) {
				Axis axis = (Axis) zeroOne(rng);
				float intercept = Axis::horizontal == axis ? yRange(rng) : xRange(rng);
				rootCellar dCepts;
				int dRoots = 0;
				if (SpeedTest::off == speedTest)
					OpCubicAxisRayHit(cubic, axis, intercept, dCepts, dRoots);
				int roots = 0;
				rootCellar cepts;
				for (auto& edge : segment.edges) {
					rootCellar edgeCepts;
					if (Axis::horizontal == axis ? 
							!OpMath::Between(edge.ptBounds.top, intercept, edge.ptBounds.bottom) :
							!OpMath::Between(edge.ptBounds.left, intercept, edge.ptBounds.right))
						continue;
					int edgeRoots = cubic.axisRayHit(axis, intercept, edge.start, edge.end, edgeCepts);
					if (SpeedTest::on == speedTest)
						continue;
					OP_ASSERT((size_t) roots + (size_t) edgeRoots <= edgeCepts.size());
					for (int index = 0; index < edgeRoots; ++index)
						cepts[roots++] = edgeCepts[index];
				}
				if (SpeedTest::on == speedTest)
					continue;
				if (roots != dRoots) { // break and trace through second time
					for (auto& edge : segment.edges) {
						rootCellar edgeCepts;
						(void) cubic.axisRayHit(axis, intercept, edge.start, edge.end, edgeCepts);
					}
					OpCubicAxisRayHit(cubic, axis, intercept, dCepts, dRoots);
					continue;
				}
				runAxisTest(cubic, intercept, cepts, dCepts, roots);
			}
			continue;
		}
		// for a bunch of t values: what is the greatest error between float and double?
		for (int test = 0; test < 100; ++test) {
			if (CubicTest::ptAtT == testType) {
	// what is the maximum error for point at t for cubics?
				float fT = t(rng);
				OpPoint pt = cubic.ptAtT(fT);
				if (SpeedTest::on == speedTest)
					continue;
				OpPoint dPt;
					OpCubicPtAtT(cubic, fT, dPt);  // double version from OpDebugDouble
				float error = findWorst(pt.x, dPt.x);
				if (worstError < error)
					recordWorst(cubic, fT, pt, dPt, error, true);
				error = findWorst(pt.y, dPt.y);
				if (worstError < error)
					recordWorst(cubic, fT, pt, dPt, error, false);
				error = findLargest(pt.x, dPt.x);
				if (largestError < error)
					recordPtAtT(cubic, fT, pt, dPt, error, true);
				error = findLargest(pt.y, dPt.y);
				if (largestError < error)
					recordPtAtT(cubic, fT, pt, dPt, error, false);
			} else {
	// what is the maximum error for axis-aligned line intersects cubic?
				// start here
				// add option to test divide cubic into edges, calling axisRayHit 'edge' version
				Axis axis = (Axis) zeroOne(rng);
				float intercept = Axis::horizontal == axis ? yRange(rng) : xRange(rng);
				rootCellar cepts;
				int roots = 0;
				if (SpeedTest::off == speedTest || CubicTest::axisRay == testType)
					roots = cubic.axisRayHit(axis, intercept, cepts);
				rootCellar dCepts;
				int dRoots = 0;
				if (SpeedTest::off == speedTest || CubicTest::doubleRay == testType)
					OpCubicAxisRayHit(cubic, axis, intercept, dCepts, dRoots);
				if (SpeedTest::on == speedTest)
					continue;
				if (roots != dRoots) { // break and trace through second time
					roots = cubic.axisRayHit(axis, intercept, cepts);
					OpCubicAxisRayHit(cubic, axis, intercept, dCepts, dRoots);
					continue;
				}
				runAxisTest(cubic, intercept, cepts, dCepts, roots);
			}
		}
	}
	if (SpeedTest::on == speedTest) {
		uint64_t endTime = OpReadTimer();
		float seconds = OpTicksToSeconds(endTime - startTime, frequency);
		if (CubicTest::ptAtT == testType)
			OpPrintOut("ptAtT:" + std::to_string(seconds) + "\n");
		else if (CubicTest::edgeRay == testType)
			OpPrintOut("edgeRay:" + std::to_string(seconds) + "\n");
		else if (CubicTest::axisRay == testType)
			OpPrintOut("axisRay:" + std::to_string(seconds) + "\n");
		else
			OpPrintOut("doubleRay:" + std::to_string(seconds) + "\n");
	} else if (CubicTest::ptAtT == testType) {
		OpDebugOut("worst:" + worst.debugDump() + "\nt:" + STR(worstT) + " pt:" + worstPt.debugDump()
				+ " dPt:" + worstDPt.debugDump() + " isX:" + STR(worstX) + " error:" + STR(worstError) + "\n");
		OpDebugOut("largest:" + largest.debugDump() + "\nt:" + STR(largestT) + " pt:" + largestPt.debugDump()
				+ " dPt:" + largestDPt.debugDump() + " error:" + STR(largestError));
	} else {
		OpDebugOut("largest:" + largest.debugDump() + "\n"
				+ "t:" + STR(largestT) + " cept:" + STR(largestCept) + " dCept:" 
				+ STR(largestDCept) + " error:" + STR(largestError) + "\n"
				+ "ptAtT:" + largest.ptAtT(largestCept).debugDump() 
				+ " dPtAtT:"+ largest.ptAtT(largestDCept).debugDump() + "\n");
		OpDebugOut("worst:" + worst.debugDump() + "\n"
				+ "xy:" + STR(worstXY) + " t:" + STR(worstT) + " pt:" + worstPt.debugDump()
				+ " dt:" + STR(worstDT) + " dPt:" + worstDPt.debugDump() 
				+ " isX:" + STR(worstX) + " error:" + STR(worstError) + "\n");
		
	}
	OpDebugOut("");
		// !!!consider: generating random 'useful' cubics?
			// since any kind of drawing app will probably allow the user to move the control points anywhere,
			// restricting the control points for testing is a bad idea
}
#endif

#if 0 // obsolete
void OpCurveTest() {
	const OpPoint pts[] = { { 100, 100 }, { 200, 100 }, { 200, 200 }, {100, 200} };
	OpCurve curve = { pts, OpType::line };
	OpLine line = { pts };
	OpQuad quad = { pts };
	OpConic conic = { pts, .5 };
	OpCubic cubic = { pts };

	// test pointCount
	ASSERT(2 == curve.pointCount());
	ASSERT(2 == line.pointCount());
	ASSERT(3 == quad.pointCount());
	ASSERT(3 == conic.pointCount());
	ASSERT(4 == cubic.pointCount());

	// test OpCurve toVertical
	const LinePts iLinePts = {{{ {1, 1}, {4, 4} }}};
	const OpPoint iQPts[] = {  { 2, 4}, {1, 3}, {3, 1} };
	OpQuad iQuad = { iQPts };
	OpCurve rotated = iQuad.toVertical(iLinePts);
	ASSERT(6 == rotated.pts[0].x);
	ASSERT(6 == rotated.pts[1].x);
	ASSERT(-6 == rotated.pts[2].x);
#if OP_DEBUG_DUMP
	curve.dump();
#endif
}

void OpLineTest() {
	// test OpLine axisRayHit
	const LinePts iLinePts = {{{ { 1, 1 }, { 4, 4 } }}};
	OpLine iLine = { iLinePts.pts[0], iLinePts.pts[1] };
	OpRoots intercepts;
	intercepts = iLine.axisRayHit(Axis::vertical, 2.5);
	ASSERT(1 == intercepts.count);
	ASSERT(.5 == intercepts.roots[0]);
	intercepts = iLine.axisRayHit(Axis::horizontal, 2.5);
	ASSERT(1 == intercepts.count);
	ASSERT(.5 == intercepts.roots[0]);


	// test OpLine interp
	const OpPoint tLinePts[] = { { 1, 2 }, { 3, 4 } };
	OpLine tLine = { tLinePts };
	float terp = tLine.interp(XyChoice::inX, .25);
	ASSERT(1.5 == terp);
	terp = tLine.interp(XyChoice::inY, .75);
	ASSERT(3.5 == terp);

	// test OpLine rayIntersect
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	OpLine l1 = { &iQPts[1] };
	const LinePts vert = {{{ { 2.5, 2 }, { 2.5, 3 } }}};
	const LinePts horz = {{{ { 4, 2.5 }, { 7, 2.5 } }}};
	intercepts = l1.rayIntersect(vert, MatchEnds::none);
	ASSERT(1 == intercepts.count);
	ASSERT(.75 == intercepts.roots[0]);
	intercepts = l1.rayIntersect(horz, MatchEnds::none);
	ASSERT(1 == intercepts.count);
	ASSERT(.25 == intercepts.roots[0]);
	intercepts = l1.rayIntersect(iLinePts, MatchEnds::none);
	ASSERT(1 == intercepts.count);
	ASSERT(.5 == intercepts.roots[0]);

	// test OpLine ptAtT
	OpPoint pt = tLine.ptAtT(0);
	ASSERT(1 == pt.x && 2 == pt.y);
	pt = tLine.ptAtT(0.25);
	ASSERT(1.5 == pt.x && 2.5 == pt.y);
	pt = tLine.ptAtT(1);
	ASSERT(3 == pt.x && 4 == pt.y);
}

void OpQuadTest() {
	// test OpQuad axisRayHit
	const OpPoint q1[] = { { 100, 100 }, { 200, 100 }, { 200, 200 } };
	OpQuad q = { q1 };
	OpRoots intercepts;
	intercepts = q.axisRayHit(Axis::vertical, 150);
	ASSERT(1 == intercepts.count);
	OpPoint pt = q.ptAtT(intercepts.roots[0]);
	ASSERT(150 == pt.x);

	intercepts = q.axisRayHit(Axis::horizontal, 150);
	ASSERT(1 == intercepts.count);
	pt = q.ptAtT(intercepts.roots[0]);
	ASSERT(150 == pt.y);

	// test OpQuad coefficients
	// !!! tested by quad axisRayHit

	OpPoint qe1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 } };
	OpPoint qe2[] = { { 1, 1 }, { 2, 2 }, { 3, 1 } };
	// test OpQuad extrema
	OpQuad qe = { qe1 };
	OpRoots extrema;
	extrema = qe.extrema(XyChoice::inX);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);
	extrema = qe.extrema(XyChoice::inY);
	ASSERT(0 == extrema.count);
	qe = { qe2 };
	extrema = qe.extrema(XyChoice::inX);
	ASSERT(0 == extrema.count);
	extrema = qe.extrema(XyChoice::inY);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);

	// test OpQuad rayIntersect
	OpRoots icepts[3];
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	OpQuad qi1 = { &iQPts[0] };
	const LinePts vert = {{{ { 2.5, 2 }, { 2.5, 3 } }}};
	const LinePts horz = {{{ { 4, 2.5 }, { 7, 2.5 } }}};
	icepts[0] = qi1.rayIntersect(vert, MatchEnds::none);
	ASSERT(1 == icepts[0].count);
	ASSERT(OpMath::Between(0.86f, icepts[0].roots[0], 0.8604f));
	icepts[1] = qi1.rayIntersect(horz, MatchEnds::none);
	ASSERT(1 == icepts[1].count);
	ASSERT(OpMath::Between(0.5811f, icepts[1].roots[0], 0.5812f));
	const LinePts iLinePts = {{{ { 1, 1 }, { 4, 4 } }}};
	icepts[2] = qi1.rayIntersect(iLinePts, MatchEnds::none);
	ASSERT(1 == icepts[2].count);
	// answer is sqrt(2)/2
	ASSERT(OpMath::Between(0.7071f, icepts[2].roots[0], 0.7072f));

	// test OpQuad monotonic
	ASSERT(!qi1.monotonic(XyChoice::inX));
	ASSERT(qi1.monotonic(XyChoice::inY));

	// test OpQuad normal
	OpVector normv = qi1.normal(0);
	OpVector rawtan = { qi1.pts[1].x - qi1.pts[0].x,
		qi1.pts[1].y - qi1.pts[0].y };
	ASSERT(normv.dx == -rawtan.dy && normv.dy == rawtan.dx);
	normv = qi1.normal(1);
	rawtan = { qi1.pts[2].x - qi1.pts[1].x,
		qi1.pts[2].y - qi1.pts[1].y };
	ASSERT(normv.dx == -rawtan.dy && normv.dy == rawtan.dx);

	// test OpQuad ptAtT
	pt = qi1.ptAtT(0);
	ASSERT(pt == iQPts[0]);
	pt = qi1.ptAtT(1);
	ASSERT(pt == iQPts[2]);
	pt = qi1.ptAtT(.5);
	SkDQuad skquad;
	skquad.set((SkPoint*) iQPts);
	SkDPoint dPt = skquad.ptAtT(.5);
	ASSERT(OpMath::Between(-0.000001f, pt.x - dPt.fX, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - dPt.fY, 0.000001f));

	// test OpQuad subDivide
	OpPtT icPtT[2] = {{ qi1.ptAtT(icepts[0].roots[0]), icepts[0].roots[0] }, 
		{ qi1.ptAtT(icepts[1].roots[0]), icepts[1].roots[0] }};
	OpCurve qdst = qi1.subDivide(icPtT[0], icPtT[1]);
	pt = qi1.ptAtT(icepts[0].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - qdst.pts[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - qdst.pts[0].y, 0.000001f));
	pt = qi1.ptAtT(icepts[1].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - qdst.pts[2].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - qdst.pts[2].y, 0.000001f));

	// test OpQuad tangent
	OpVector tanv = qi1.tangent(0);
	rawtan = { qi1.pts[1].x - qi1.pts[0].x,
		                qi1.pts[1].y - qi1.pts[0].y };
	ASSERT(tanv.dx == rawtan.dx && tanv.dy == rawtan.dy);
	tanv = qi1.tangent(1);
	rawtan = { qi1.pts[2].x - qi1.pts[1].x,
		qi1.pts[2].y - qi1.pts[1].y };
	ASSERT(tanv.dx == rawtan.dx && tanv.dy == rawtan.dy);
}

void OpConicTest( ) {
	// test OpConic axisRayHit
	OpRoots intercepts;
	const OpPoint q1[] = { { 100, 100 }, { 200, 100 }, { 200, 200 } };
	OpConic c = { q1, sqrtf(2)/2 };
	intercepts = c.axisRayHit(Axis::vertical, 150);
	ASSERT(1 == intercepts.count);
	OpPoint pt = c.ptAtT(intercepts.roots[0]);
	ASSERT(OpMath::Between(-0.0001f, pt.x - 150, 0.0001f));

	// test OpConic coefficients
	// !!! tested by conic axisRayHit

	// test OpConic denominator
	// !!! tested by ptatt
	
	// test OpConic derivative_coefficients
	// !!! tested by extrema
	
	// test OpConic extrema
//	OpPoint ce1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 } };
	OpPoint ce2[] = { { 1, 1 }, { 2, 2 }, { 3, 1 } };
	OpPoint qe1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 } };
//	OpPoint qe2[] = { { 1, 1 }, { 2, 2 }, { 3, 1 } };
	OpConic ce = { qe1, sqrtf(2)/2 };
	OpRoots extrema;
	extrema = ce.extrema(XyChoice::inX);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);
	extrema = ce.extrema(XyChoice::inY);
	ASSERT(0 == extrema.count);
	ce = { ce2, sqrtf(2) / 2 };
	extrema = ce.extrema(XyChoice::inX);
	ASSERT(0 == extrema.count);
	extrema = ce.extrema(XyChoice::inY);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);

	// test OpConic rayIntersect
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	OpConic ci1 = { &iQPts[0], sqrtf(2) / 2 };
	const LinePts vert = {{{ { 2.5, 2 }, { 2.5, 3 } }}};
	const LinePts horz = {{{ { 4, 2.5 }, { 7, 2.5 } }}};
	OpRoots icepts[3];
	icepts[0] = ci1.rayIntersect(vert, MatchEnds::none);
	ASSERT(1 == icepts[0].count);
	ASSERT(OpMath::Between(0.817f, icepts[0].roots[0], 0.818f));
	icepts[1] = ci1.rayIntersect(horz, MatchEnds::none);
	ASSERT(1 == icepts[1].count);
	ASSERT(OpMath::Between(0.5581f, icepts[1].roots[0], 0.5582f));
	const LinePts iLinePts = {{{ { 1, 1 }, { 4, 4 } }}};
	icepts[2] = ci1.rayIntersect(iLinePts, MatchEnds::none);
	ASSERT(1 == icepts[2].count);
	ASSERT(OpMath::Between(0.6589f, icepts[2].roots[0], 0.659f));

	// test OpConic monotonic
	ASSERT(!ci1.monotonic(XyChoice::inX));
	ASSERT(ci1.monotonic(XyChoice::inY));

	// test OpConic normal
	OpVector normv = ci1.normal(0);
	OpVector rawtan = { ci1.pts[1].x - ci1.pts[0].x,
			  ci1.pts[1].y - ci1.pts[0].y };
	OpVector normaledv = normv.normalize();
	OpVector normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledv.dx + normaledr.dy, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledv.dy - normaledr.dx, .0001f));
	normv = ci1.normal(1);
	rawtan = { ci1.pts[2].x - ci1.pts[1].x,
			   ci1.pts[2].y - ci1.pts[1].y };
	normaledv = normv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledv.dx + normaledr.dy, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledv.dy - normaledr.dx, .0001f));

	// test OpConic numerator
	// !!! tested by ptatt
	 
	// test OpConic ptAtT
	pt = ci1.ptAtT(0);
	ASSERT(pt == iQPts[0]);
	pt = ci1.ptAtT(1);
	ASSERT(pt == iQPts[2]);
	pt = ci1.ptAtT(.5);
	SkDConic skconic;
	skconic.set((SkPoint*)iQPts, sqrtf(2) / 2);
	SkDPoint dPt = skconic.ptAtT(.5);
	ASSERT(OpMath::Between(-0.000001f, pt.x - dPt.fX, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - dPt.fY, 0.000001f));

	// test OpConic subDivide
	OpPtT icPtT[2] = { { ci1.ptAtT(icepts[0].roots[0]), icepts[0].roots[0] },
		{ ci1.ptAtT(icepts[1].roots[0]), icepts[1].roots[0] } };
	OpCurve cdst = ci1.subDivide(icPtT[0], icPtT[1]);
	pt = ci1.ptAtT(icepts[0].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst.pts[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst.pts[0].y, 0.000001f));
	pt = ci1.ptAtT(icepts[1].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst.pts[2].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst.pts[2].y, 0.000001f));

	// test OpConic tangent
	OpVector tanv = ci1.tangent(0);
	rawtan = { ci1.pts[1].x - ci1.pts[0].x,
			   ci1.pts[1].y - ci1.pts[0].y };
	OpVector normaledt = tanv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledt.dx - normaledr.dx, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledt.dy - normaledr.dy, .0001f));
	tanv = ci1.tangent(1);
	rawtan = { ci1.pts[2].x - ci1.pts[1].x,
			   ci1.pts[2].y - ci1.pts[1].y };
	normaledt = tanv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledt.dx - normaledr.dx, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledt.dy - normaledr.dy, .0001f));

}

void OpCubicTest() {
	// test OpCubic axisRayHit
	const OpPoint c1[] = { { 100, 100 }, {150, 100}, { 200, 150 }, { 200, 200 } };
	OpCubic c = { c1 };
	OpRoots intercepts;
	intercepts = c.axisRayHit(Axis::vertical, 150);
	ASSERT(1 == intercepts.count);
	OpPoint pt = c.ptAtT(intercepts.roots[0]);
	ASSERT(OpMath::Between(-.0001f, pt.x - 150, .0001f));

	intercepts = c.axisRayHit(Axis::horizontal, 150);
	ASSERT(1 == intercepts.count);
	pt = c.ptAtT(intercepts.roots[0]);
	ASSERT(OpMath::Between(-.0001f, pt.y - 150, .0001f));

	// test OpCubic coefficients
	// !!! tested by axisRayHit

	// test OpCubic extrema
	OpPoint ce1[] = { { 1, 1 }, { 2, 2 }, { 2, 3 }, { 1, 4 } };
	OpPoint ce2[] = { { 1, 1 }, { 2, 2 }, { 3, 2 }, { 4, 1 } };
	OpCubic ce = { ce1 };
	OpRoots extrema;
	extrema = ce.extrema(XyChoice::inX);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);
	extrema = ce.extrema(XyChoice::inY);
	ASSERT(0 == extrema.count);
	ce = { ce2 };
	extrema = ce.extrema(XyChoice::inX);
	ASSERT(0 == extrema.count);
	extrema = ce.extrema(XyChoice::inY);
	ASSERT(1 == extrema.count);
	ASSERT(0.5 == extrema.roots[0]);

	// test OpCubic inflections
	OpPoint ci1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 }, { 2, 4 } };
	OpCubic ci = { ci1 };
	OpRoots inflections;
	inflections = ci.inflections();
	ASSERT(1 == inflections.count);
	ASSERT(0.5 == inflections.roots[0]);

	// test OpCubic interp
	// !!! tested by subdivide

	// test OpCubic rayIntersect
	OpRoots icepts[3];
	const OpPoint iCPts[] = { { 2, 4 }, { 1, 3 }, { 1, 2 }, { 3, 1 } };
	ci = { &iCPts[0] };
	const LinePts vert = {{{ { 2.5, 2 }, { 2.5, 3 } }}};
	const LinePts horz = {{{ { 4, 2.5 }, { 7, 2.5 } }}};
	icepts[0] = ci.rayIntersect(vert, MatchEnds::none);
	ASSERT(1 == icepts[0].count);
	ASSERT(OpMath::Between(0.9084f, icepts[0].roots[0], 0.9085f));
	pt = ci.ptAtT(icepts[0].roots[0]);
	ASSERT(OpMath::Between(-.0001f, pt.x - 2.5, .0001f));
	icepts[1] = ci.rayIntersect(horz, MatchEnds::none);
	ASSERT(1 == icepts[1].count);
	ASSERT(OpMath::Between(0.5f, icepts[1].roots[0], 0.5001f));
	pt = ci.ptAtT(icepts[1].roots[0]);
	ASSERT(OpMath::Between(-.0001f, pt.y - 2.5, .0001f));
	const LinePts iLinePts = {{{ { 1, 1 }, { 4, 4 } }}};
	icepts[2] = ci.rayIntersect(iLinePts, MatchEnds::none);
	ASSERT(1 == icepts[2].count);
	ASSERT(OpMath::Between(0.7320f, icepts[2].roots[0], 0.7321f));
	pt = ci.ptAtT(icepts[2].roots[0]);
	ASSERT(OpMath::Between(-.0001f, pt.y - pt.x, .0001f));
	ASSERT(OpMath::Between(1, pt.y, 4));

	// test OpCubic monotonic
	ASSERT(!ci.monotonic(XyChoice::inX));
	ASSERT(ci.monotonic(XyChoice::inY));

	// test OpCubic normal
	OpVector normv = ci.normal(0);
	OpVector rawtan = { ci.pts[1].x - ci.pts[0].x, ci.pts[1].y - ci.pts[0].y };
	OpVector normaledv = normv.normalize();
	OpVector normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledv.dx + normaledr.dy, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledv.dy - normaledr.dx, .0001f));
	normv = ci.normal(1);
	rawtan = { ci.pts[3].x - ci.pts[2].x, ci.pts[3].y - ci.pts[2].y };
	normaledv = normv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledv.dx + normaledr.dy, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledv.dy - normaledr.dx, .0001f));

	// test OpCubic ptAtT
	pt = ci.ptAtT(0);
	ASSERT(pt == iCPts[0]);
	pt = ci.ptAtT(1);
	ASSERT(pt == iCPts[3]);
	pt = ci.ptAtT(.5);
	SkDCubic skcubic;
	skcubic.set((SkPoint*)iCPts);
	SkDPoint dPt = skcubic.ptAtT(.5);
	ASSERT(OpMath::Between(-0.000001f, pt.x - dPt.fX, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - dPt.fY, 0.000001f));

	// test OpCubic subDivide
	OpPtT icPtT[2] = { { ci.ptAtT(icepts[0].roots[0]), icepts[0].roots[0] },
		{ ci.ptAtT(icepts[1].roots[0]), icepts[1].roots[0] } };
	OpCurve cdst = ci.subDivide(icPtT[0], icPtT[1]);
	pt = ci.ptAtT(icepts[0].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst.pts[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst.pts[0].y, 0.000001f));
	pt = ci.ptAtT(icepts[1].roots[0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst.pts[3].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst.pts[3].y, 0.000001f));

	// test OpCubic tangent
	OpVector tanv = ci.tangent(0);
	rawtan = { ci.pts[1].x - ci.pts[0].x, ci.pts[1].y - ci.pts[0].y };
	OpVector normaledt = tanv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledt.dx - normaledr.dx, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledt.dy - normaledr.dy, .0001f));
	tanv = ci.tangent(1);
	rawtan = { ci.pts[3].x - ci.pts[2].x, ci.pts[3].y - ci.pts[2].y };
	normaledt = tanv.normalize();
	normaledr = rawtan.normalize();
	ASSERT(OpMath::Between(-.0001f, normaledt.dx - normaledr.dx, .0001f));
	ASSERT(OpMath::Between(-.0001f, normaledt.dy - normaledr.dy, .0001f));
}
#endif

#include "OpContour.h"
#include "OpEdge.h"
#include "PathOps.h"

#if 0
void TestPathOps(OpInPath& op1, OpInPath& op2, OpOperator operation, OpOutPath& opOut) {
	OpDebugData debugData(false);
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, operation, opOut
			OP_DEBUG_PARAMS(debugData));
	OP_ASSERT(didIt);

}

bool OpPathOpsTest1(const SkPath& one, const SkPath& two, SkPath* result) {
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(result);
	TestPathOps(op1, op2, OpOperator::Intersect, opOut);
	result->dump();
	OpDebugOut("success!\n");
	return true;
}

void OpTestXor() {
	SkPath one, two, result;
	SkRect r1 = { 1, 2, 5, 6 };
	SkRect r2 = { 3, 4, 7, 8 };
	one.addRect(r1, SkPathDirection::kCCW);
	two.addRect(r2, SkPathDirection::kCCW);
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(&result);
	TestPathOps(op1, op2, OpOperator::ExclusiveOr, opOut);
	result.dump();
	OpDebugOut("");
}

void OpTestRect() {
	SkPath one, two, result;
	SkRect r1 = { 1, 2, 5, 6 };
	SkRect r2 = { 3, 4, 7, 8 };
#if OP_DEBUG
	SkRect r3 = { 3, 4, 5, 6 };
#endif
	std::array<SkPathDirection, 2> ccs = { SkPathDirection::kCW, SkPathDirection::kCCW };
	for (SkPathDirection c1 : ccs) {
		for (SkPathDirection c2 : ccs) {
			one.reset();
			two.reset();
			one.addRect(r1, c1);
			two.addRect(r2, c2);
			result.reset();
			(void) OpPathOpsTest1(one, two, &result);
			OP_ASSERT(result.getBounds() == r3);
			result.reset();
			(void)OpPathOpsTest1(two, one, &result);
			OP_ASSERT(result.getBounds() == r3);
		}
	}
}

void OpTestCoincidence() {
	SkPath one, two, result;
	SkRect r1 = { 1, 2, 5, 6 };
	SkRect r2 = { 1, 2, 7, 8 };
	one.addRect(r1);
	two.addRect(r2);
	(void)OpPathOpsTest1(one, two, &result);
	OP_ASSERT(result.getBounds() == r1);
}
#endif

void OpTestOpEdgesConcidenceCheck() {
#if 0	// broken: find need before fixing
	for (float a = 2; a < 10; a += 2) {
		for (float b = 2; b < 10; b += 2) {
			if (a == b)
				continue;
			for (float c = 2; c < 10; c += 2) {
				for (float d = 2; d < 10; d += 2) {
					if (c == d)
						continue;
					CurvePts ab = {{ {{ a, 8 }, { b, 8 }} }, 1 };
					CurvePts cd = {{ {{ c, 8 }, { d, 8 }} }, 1 };
					OpContours contours(OpOperator::Intersect);
					contours.left = OpFillType::evenOdd;
					OpContour contour(&contours, OpOperand::left);
					OpSegment seg(ab, OpType::line  
							OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, &contour));
					OpSegment oSeg(cd, OpType::line  
							OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, &contour));
					OpEdge edge(&seg, { ab.pts[0], 0}, { ab.pts[1], 1 }, false  
							OP_DEBUG_PARAMS(EDGE_MAKER(opTest), nullptr, nullptr));
					OpEdge oEdge(&oSeg, { cd.pts[0], 0 }, { cd.pts[1], 1 }, false  
							OP_DEBUG_PARAMS(EDGE_MAKER(opTest), nullptr, nullptr));
					OpEdges::CoincidentCheck(oEdge, edge);
					float _ab[2] = { a, b};
					float _cd[2] = { c, d};
					if (_ab[0] > _ab[1]) std::swap(_ab[0], _ab[1]);
					if (_cd[0] > _cd[1]) std::swap(_cd[0], _cd[1]);
					float min = std::max(_ab[0], _cd[0]);
					float max = std::min(_ab[1], _cd[1]);
					if (min > max) {
						OP_ASSERT(!seg.intersections.size());
						OP_ASSERT(!oSeg.intersections.size());
						continue;
					}
					if (min == max) {
						OP_ASSERT(1 == seg.intersections.size());
						OP_ASSERT(seg.intersections[0]->ptT.t == 0 || seg.intersections[0]->ptT.t == 1);
						OP_ASSERT(1 == oSeg.intersections.size());
						OP_ASSERT(oSeg.intersections[0]->ptT.t == 0 || oSeg.intersections[0]->ptT.t == 1);
					} else {
						OP_ASSERT(2 == seg.intersections.size());
						OP_ASSERT(2 == oSeg.intersections.size());
					}
					OP_DEBUG_CODE(seg.debugValidate());
					OP_DEBUG_CODE(oSeg.debugValidate());
				}
			}
		}
	}
#endif
}

struct WindStateTest {
	int wind;
	int sum;
	WindState state;
};

void OpTest_WindState() {
#if 0	// broken: find need before fixing
	WindStateTest tests[] = {
		{ 0, 0, WindState::zero },
		{ 0, 1, WindState::one },
		{ 1, 0, WindState::flipOn },
		{ 1, 1, WindState::flipOff },
		{ 1, 2, WindState::one },
		{ 2, 0, WindState::flipOn },
	};
	OpContours contours(OpOperator::Intersect);
	contours.left = OpFillType::winding;
	for (size_t index = 0; index < ARRAY_COUNT(tests); ++index) {
		const WindStateTest& test = tests[index];
		if (!(test.state == contours.windState(test.wind, test.sum, OpOperand::left)))
			OP_ASSERT(0);
	}
#endif
}

#if 0
struct WindZeroTest {
	int wind;
	int sum;
	WindZero state;
};

void OpTest_WindZero() {
	OpPtT data2[] = {
		{ { 1, 2 }, 0 },
		{ { 3, 4 }, 1 }
	};
	OpEdge edge2(data2);
	WindZeroTest tests[] = {
		{ 0, 0, WindZero::noFlip },
		{ 0, 1, WindZero::noFlip },
		{ 1, 0, WindZero::normal },
		{ 1, 1, WindZero::opp },
		{ 1, 2, WindZero::noFlip },
		{ 2, 0, WindZero::normal },
	};
	for (size_t index = 0; index < ARRAY_COUNT(tests); ++index) {
		const WindZeroTest& test = tests[index];
		edge2.winding.left = test.wind;
		edge2.sum.left = test.sum;
		edge2.winding.right = 1;
		edge2.sum.right = 1;
		OP_ASSERT(test.state == edge2.windZero(OpOperand::left));
	}
}
#endif

void OpTest_EdgeZero() {
#if 0	// broken: find need before fixing
	OpPoint data1[][2] = {
		{ { 1, 2 }, { 3, 4 } },
		{ { 3, 4 }, { 1, 2 } },
	};
	OpPoint data2[][2] = {
		{ { 1, 2 }, { -3, 4 } },
		{ { -3, 4 }, { 1, 2 } },
	};
	for (unsigned i1 = 0; i1 < ARRAY_COUNT(data1); ++i1) {
		for (int w1 = 0; w1 < 16; ++w1) {
			for (unsigned i2 = 0; i2 < ARRAY_COUNT(data2); ++i2) {
				for (int w2 = 0; w2 < 16; ++w2) {
					OpContours contours(OpOperator::Intersect);
					contours.left = OpFillType::winding;
					OP_DEBUG_CODE(debugGlobalContours = &contours);
					contours.contours.emplace_back(&contours, OpOperand::left);
					OpContour* head = &contours.contours.back();
//					opDebugImage.init(nullptr, nullptr);
					head->addLine(data1[i1]);
					head->addLine(data2[i2]);
					OpSegment& seg1 = head->segments.front();
//					opDebugImage.setFocus(seg1.ptBounds);
					seg1.addSegSect(OpPtT(data1[0][0], 0)
							OP_DEBUG_PARAMS(SECT_MAKER(opTestEdgeZero1), SectReason::test, nullptr));
					seg1.addSegSect(OpPtT(data1[0][1], 1)
							OP_DEBUG_PARAMS(SECT_MAKER(opTestEdgeZero2), SectReason::test, nullptr));
					seg1.makeEdges();
					OpEdge& edge1 = seg1.edges[0];
					if (!setWinding(edge1, w1))
						continue;
					OpSegment& seg2 = *std::next(head->segments.begin());
					seg2.addSegSect(OpPtT(data2[0][0], 0)
							OP_DEBUG_PARAMS(SECT_MAKER(opTestEdgeZero3), SectReason::test, nullptr));
					seg2.addSegSect(OpPtT(data2[0][1], 1)
							OP_DEBUG_PARAMS(SECT_MAKER(opTestEdgeZero4), SectReason::test, nullptr));
					seg2.makeEdges();
//					opDebugImage.setFocus(seg2.ptBounds);
					OpEdge& edge2 = seg2.edges[0];
					if (!setWinding(edge2, w2))
						continue;
//					edge1.debugImageAdd();
//					edge2.debugImageAdd();
//					toggleNormals();
// 					OpOperand side1Op, side2Op;
//					WindZero zeroSide1 = edge1.zeroSide(&side1Op);
//					WindZero zeroSide2 = edge2.zeroSide(&side2Op);
				}
				OpDebugOut("");
			}
			OpDebugOut("");
		}
		OpDebugOut("");
	}
	OpDebugOut("");
#endif
}

#if 0
void OpTestQuadLine() {
	SkPath one, two, result;
	one.moveTo(0, 0);
	one.lineTo(2, 0);
	one.lineTo(1, 2);
	one.close();
	two.moveTo(0, 2);
	two.quadTo(0, 0, 2, 0);
	two.lineTo(2, 2);
	two.close();
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(&result);
	TestPathOps(op1, op2, OpOperator::Intersect, opOut);
	result.dump();
	OpDebugOut("");
}

void OpTestQuadQuad() {
	SkPath one, two, result;
	one.moveTo(0, 0);
	one.quadTo(1, 2, 2, 0);
	one.close();
	two.moveTo(0, 2);
	two.quadTo(0, 0, 2, 0);
	two.lineTo(2, 2);
	two.close();
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(&result);
	TestPathOps(op1, op2, OpOperator::Intersect, opOut);
	result.dump();
	OpDebugOut("");
}

void OpTestQuadCoin() {
	SkPath one, two, result;
	one.moveTo(0, 0);
	one.quadTo(1, 2, 2, 2);
	one.quadTo(3, 2, 4, 0);
	one.close();
	two.moveTo(0, 0);
	two.quadTo(2, 4, 4, 0);
	two.close();
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(&result);
	TestPathOps(op1, op2, OpOperator::Intersect, opOut);
	result.dump();
	OpDebugOut("");
}

void OpTestQuadCoin2() {
	SkPath one, two, result;
	one.moveTo(0, 0);
	one.quadTo(1, 2, 2, 2);
	one.quadTo(3, 2, 4, 0);
	one.close();
	two.moveTo(4, 0);
	two.quadTo(2, 4, 0, 0);
	two.close();
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(&result);
	TestPathOps(op1, op2, OpOperator::Intersect, opOut);
	result.dump();
	OpDebugOut("");
}
#endif

#if 0
void onetest() {
	OpPoint data[] = { 
		  { OpDebugBitsToFloat(0x42f7639d), OpDebugBitsToFloat(0x4116d2c4) }  // {123.694557, 9.426456}
		, { OpDebugBitsToFloat(0x42f7639d), OpDebugBitsToFloat(0x4116d2c3) }  // {123.694557, 9.426455}
		, { OpDebugBitsToFloat(0x42f7639b), OpDebugBitsToFloat(0x4116d2c4) }  // {123.694542, 9.426456}
		, { OpDebugBitsToFloat(0x42f7639b), OpDebugBitsToFloat(0x4116d2c3) }  // {123.694542, 9.426455}
	}; 
	OpCurve curve(data, OpType::cubic);
	curve.center(Axis::vertical, 123.694550f);
}
#endif

#if 0
OpPoint testMinMax(OpPoint x, OpPoint y) {
	OpPoint a = OpPoint::Min1(x, y);
	OpPoint b = OpPoint::Min2(x, y);
	OpPoint c = x.min1(y);
	OpPoint d = x.min2(y);
	return a + b + c + d;
}

#include <stdlib.h>
#endif

void SkConicSubdivide();
void CCTestKey(uint32_t );
void CCTest();

#if 01
#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/UnaryWinding.h"
#include "OpSegments.h"

PathOpsV0Lib::CurveType testLineType = (PathOpsV0Lib::CurveType) 0;  // unset

void testOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, PathOpsV0Lib::PathOutput output) {
}

void testNoEmptyPath(PathOpsV0Lib::PathOutput ) {
}

PathOpsV0Lib::Curve testMakeLine(PathOpsV0Lib::Curve c) {
    c.type = testLineType;
    c.size = sizeof(OpPoint) * 2;
    return c;
}

PathOpsV0Lib::CurveType getTestLineType(PathOpsV0Lib::Curve ) {
    return testLineType;
}

void LineCoincidenceTest() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext({nullptr, 0});
    SetContextCallBacks(context, testNoEmptyPath, testMakeLine, getTestLineType, maxSignSwap,
			maxDepth, maxSplits, maxLimbs);

    testLineType = SetCurveCallBacks(context, lineAxisT, nullptr, nullptr, 
            nullptr, noBounds, testOutput, nullptr, nullptr,
            lineTangent, nullptr, linePtAtT,
            nullptr, nullptr, lineSubDivide, lineXYAtT,
			lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(noDebugScale)
            OP_DEBUG_DUMP_PARAMS(noDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(noAddToSkPathFunc)
    );

    Contour* contour = CreateContour({context, nullptr, 0});
    SetWindingCallBacks(contour, unaryWindingAddFunc, unaryWindingKeepFunc, 
            unaryWindingSubtractFunc, unaryWindingVisibleFunc, unaryWindingZeroFunc 
			OP_DEBUG_PARAMS(noDebugBitOper)
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, noDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(noWindingImageOutFunc, noNativePathFunc,
                    noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
    );
    int windingData[] = { 1 };
    AddWinding addWinding { contour, windingData, sizeof(windingData) };
    constexpr size_t lineSize = sizeof(OpPoint) * 2;
	auto OpPtHex = [](uint32_t x, uint32_t y) {
		return OpPoint(OpDebugBitsToFloat(x), OpDebugBitsToFloat(y));
	};
	auto OpPtTHex = [OpPtHex](uint32_t x, uint32_t y, uint32_t t) {
		return OpPtT(OpPtHex(x, y), OpDebugBitsToFloat(t));
	};
	struct CoinTest {
		LinePts seg;
		LinePts opp;
		OpPtT sStart;
		OpPtT sEnd;
		OpPtT oStart;
		OpPtT oEnd;
		bool isCoin;
	};
	CoinTest tests[] = {
		{{ OpPoint(2.28779936f, 4.76104975f), OpPoint(2.28780055f, 4.76105499f) },
		 { OpPoint(2.28780055f, 4.76105499f), OpPoint(2.28808880f, 4.76002407f) },
		   OpPtT(), OpPtT(), OpPtT(), OpPtT(), false },

		{{ OpPoint(1, 1), OpPoint(9, 9) },
		 { OpPoint(1 + OpEpsilon, 1), OpPoint(9 + OpEpsilon * 9, 9) },
		   OpPtTHex(0x3f800001, 0x3f800000, 0x32000000), 
		   OpPtTHex(0x41100000, 0x41100000, 0x3f800000), 
		   OpPtTHex(0x3f800001, 0x3f800000, 0x00000000), 
		   OpPtTHex(0x41100000, 0x41100000, 0x3f7ffffe), true },

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(2, 4), OpPoint(4,  8) },
		 { OpPoint(2, 4), 0     }, { OpPoint(4, 8), 2.f/3 }, 
		 { OpPoint(2, 4), 0     }, { OpPoint(4, 8), 1     }, true }, 

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(4, 8), OpPoint(2,  4) },
		 { OpPoint(2, 4), 0     }, { OpPoint(4, 8), 2.f/3 }, 
		 { OpPoint(2, 4), 1     }, { OpPoint(4, 8), 0     }, true }, 

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(5, 10), OpPoint(3, 6) },
		 { OpPoint(3, 6), 1.f/3 }, { OpPoint(5, 10), 1    }, 
	 	 { OpPoint(3, 6), 1     }, { OpPoint(5, 10), 0    }, true }, 

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(3, 6), OpPoint(5, 10) },
		 { OpPoint(3, 6), 1.f/3 }, { OpPoint(5, 10), 1    }, 
	 	 { OpPoint(3, 6), 0     }, { OpPoint(5, 10), 1    }, true }, 

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(7, 14), OpPoint(3, 6) },
		 { OpPoint(3, 6), 1.f/3 }, { OpPoint(5, 10), 1    }, 
	 	 { OpPoint(3, 6), 1     }, { OpPoint(5, 10),  .5  }, true }, 

		{{ OpPoint(2, 4), OpPoint(5, 10) }, { OpPoint(3, 6), OpPoint(7, 14) },
		 { OpPoint(3, 6), 1.f/3 }, { OpPoint(5, 10), 1   }, 
	 	 { OpPoint(3, 6), 0     }, { OpPoint(5, 10),  .5  }, true }, 

		{{ OpPoint(1, 0), OpPoint(0, 1) },
		 { OpPtHex(0x3f000000, 0x3f400001), OpPtHex(0x3f000000, 0x3f400000) },
		   OpPtT(), OpPtT(), OpPtT(), OpPtT(), false },
	};
	// for lines that should be detected:
	// start with one line. pick four random t values. assert that the overlapping t ranges match
	for (size_t index = 0; index < ARRAY_COUNT(tests); index++) {
		ResetContour(contour);
		LinePts& line1 = tests[index].seg;
		LinePts& line2 = tests[index].opp;
		Add({ &line1.pts[0], lineSize, testLineType }, addWinding );
		Add({ &line2.pts[0], lineSize, testLineType }, addWinding );
		OpSegment* test1 = &((OpContour*) contour)->segments[0];
		OpSegment* test2 = &((OpContour*) contour)->segments[1];
		IntersectResult result = OpSegments::LineCoincidence(test1, test2);
		OP_ASSERT((IntersectResult::yes == result) == tests[index].isCoin);
		if (IntersectResult::yes == result) {
			OP_ASSERT(test1->sects.i[0]->ptT == tests[index].sStart);
			OP_ASSERT(test1->sects.i[1]->ptT == tests[index].sEnd);
			OP_ASSERT(test2->sects.i[0]->ptT == tests[index].oStart);
			OP_ASSERT(test2->sects.i[1]->ptT == tests[index].oEnd);
		}
	}
}
#endif

extern void TestPath2D(bool debugIt);
extern void testFrame();

void OpTest(bool terminateEarly) {
//	testFrame();
//	TestPath2D(false);
//	LineCoincidenceTest();
#if 0
	CCTestKey(0x09b54c61);
	CCTest();
#endif

//	SkConicSubdivide();
#if 0
	OpCubicErrorTest(CubicTest::ptAtT, SpeedTest::on);
	OpCubicErrorTest(CubicTest::edgeRay, SpeedTest::on);
	OpCubicErrorTest(CubicTest::axisRay, SpeedTest::on);
	OpCubicErrorTest(CubicTest::doubleRay, SpeedTest::on);
	if (terminateEarly)
		exit(0);
#endif
	runTests();
	if (terminateEarly)
		exit(0);
	// !!! for now, just run tests to verify operation. Circle back around and 
	// write internal tests later
//	run_all_tests3();
//	run_all_tests2();  
	OpTest_WindState();
//	OpTest_WindZero();
	OpTest_EdgeZero();
//	OpPoint pt = testMinMax({ (float) rand(), (float)rand() }, { (float)rand(), (float)rand() });
//	OpDebugOut(std::to_string(pt.x));
//	OP_DEBUG_CODE(onetest());
//	OpTestQuadCoin2();
//	OpTestQuadCoin();
//	OpTestQuadQuad();
	OpMathTest();
//	OpCurveTest();
//	OpLineTest();
//	OpQuadTest();
//	OpConicTest();
//	OpCubicTest();
	OpTestOpEdgesConcidenceCheck();
//	OpTestQuadLine();
//	OpTestCoincidence();
//	OpTestXor();
//	OpTestRect();
}

#include "include/core/SkCanvas.h"

#if 0
static void drawPath(SkCanvas* canvas, const SkPath& path) {
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	canvas->drawPath(path, paint);
}
#endif

#include "src/pathops/SkIntersections.h"
#include "src/pathops/SkPathOpsConic.h"
#include "src/pathops/SkPathOpsLine.h"

void SkConicDraw(SkCanvas* canvas) {
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	const SkPoint svert[] = { { 2.5, 1 }, { 2.5, 3 } };
	const SkPoint shorz[] = { { 4, 2.5 }, { 7, 2.5 } };
	const SkPoint siQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	const SkPoint siLinePts[] = { { 1, 1 }, { 4, 4 } };
	SkDConic dc;
	SkDLine dl;
	dc.set(siQPts, sqrtf(2) / 2);
	dl.set(svert);
	SkIntersections lci;
	double sroots[2];
	int sresult = SkIntersections::VerticalIntercept(dc, 2.5, sroots);
	if (sresult) {
		SkDPoint p = dc.ptAtT(sroots[0]);
		canvas->drawCircle(p.fX, p.fY, 0.2f, paint);
	}
	dl.set(shorz);
	sresult = lci.intersectRay(dc, dl);
	if (sresult) {
		SkDPoint p = lci.pt(0);
		canvas->drawCircle(p.fX, p.fY, 0.2f, paint);
	}
	dl.set(siLinePts);
	sresult = lci.intersectRay(dc, dl);
	if (sresult) {
		SkDPoint p = lci.pt(0);
		canvas->drawCircle(p.fX, p.fY, 0.2f, paint);
	}

}

#if 0
void SkConicSubdivide() {
	OpCurve c;
	OpPoint ctrls[2] = {{-4, 2}};
	c.set(OpPoint(-4, 6), ctrls, OpPoint(0, 2), 3, OpType::conic, sqrtf(2) / 2);
	OpPtT s1 = { {-4, 6}, 0 };
	OpPtT s2 = { {-1.519868, 2.3}, 0.453012 };
	OpCurve result = c.subDivide(s1, s2);
	const SkPoint siQPts[] = { { -4, 6 }, { -4, 2 }, { 0, 2 } };
	SkDConic dc;
	dc.set(siQPts, sqrtf(2) / 2);
	SkDConic skResult = dc.subDivide(0, 0.453012);
	OpDebugOut(result.debugDump(DebugLevel::normal, DebugBase::dec) + "\n");
	OpDebugOut(STR((float)skResult.fPts[1].fX) + ", " 
			+ STR((float)skResult.fPts[1].fY) + "w:" + STR(skResult.fWeight) + "\n");
}

#include "include/core/SkFont.h"

void cubics44dDraw(SkCanvas* canvas) {
	SkPoint pts[] = { { 3, 4 }, { 2, 5 }, { 3, 1 }, { 6, 2 } };

	SkPath path;
	path.moveTo(pts[0]);
	path.cubicTo(pts[1], pts[2], pts[3]);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	OpCubic cubic((const OpPoint*)pts);
	OpPointBounds bounds = cubic.ptBounds();
	SkRect r = { bounds.left, bounds.top, bounds.right, bounds.bottom };
	paint.setColor(SK_ColorLTGRAY);
	canvas->drawRect(r, paint);
	paint.setColor(SK_ColorDKGRAY);
	for (int i = 0; i < 4; i++)
		canvas->drawCircle(pts[i].fX, pts[i].fY, 0.04f, paint);
	paint.setColor(SK_ColorBLACK);
	canvas->drawPath(path, paint);
	SkFont labelFont(nullptr, .3f, 1, 0);
	SkPaint textPaint;
	textPaint.setAntiAlias(true);
	char label[80];
	snprintf(label, sizeof(label), "%g", r.fLeft);
	canvas->drawString(SkString(label), r.left() - .8, r.centerY(), labelFont, textPaint);
	snprintf(label, sizeof(label), "%g", r.fRight);
	canvas->drawString(SkString(label), r.right() + .1, r.centerY(), labelFont, textPaint);
	snprintf(label, sizeof(label), "%g", r.fTop);
	canvas->drawString(SkString(label), r.centerX(), r.top() - .1, labelFont, textPaint);
	snprintf(label, sizeof(label), "%g", r.fBottom);
	canvas->drawString(SkString(label), r.centerX(), r.bottom() + .3, labelFont, textPaint);
	for (int i = 0; i < 4; i++) {
		snprintf(label, sizeof(label), "(%g, %g)", pts[i].fX, pts[i].fY);
		canvas->drawString(SkString(label), pts[i].fX + .1, pts[i].fY + .05, labelFont, textPaint);
	}
	OpPoint data[] = {
		{ OpDebugBitsToFloat(0x402f3d1c), OpDebugBitsToFloat(0x40852e3e) }, // {2.7381, 4.16189}
		{ OpDebugBitsToFloat(0x402be4f9), OpDebugBitsToFloat(0x40852e3e) }, // {2.68585, 4.16189}
		{ OpDebugBitsToFloat(0x4029b581), OpDebugBitsToFloat(0x40840f04) }, // {2.6517, 4.12683}
		{ OpDebugBitsToFloat(0x4028aea9), OpDebugBitsToFloat(0x4082171f) }  // {2.63566, 4.06532}
	};  // type:cubic t:  0.112702, 0.207849
}
#endif

#include "OpCurveCurve.h"

// constexpr int maxOverlaps = 16;

void SetupDebugImage() {
#if OP_DEBUG_IMAGE
	OpDebugImage::init();
	hideOperands();
	hideSegmentEdges();
	showEdges();
	showPoints();
	showIDs();
#endif
}

#if 0
// Look for edge pairs which are close but do not cross, and which are difficult to divide into
// nonoverlapping pieces. See if it is possible to know that some edges do not intersect if their
// end normals increase in distance.
static bool checkNormals(uint32_t key, int testDepth, OpCurveCurve& cc, const OpCurveCurve& ref) {
	float priorDist = OpNaN;  // require three consecutive distances
	float lastDist = OpNaN;
	for (OpEdge* edge : cc.edgeCurves.c) {
		if (!edge->ccOverlaps) {
			priorDist = lastDist = OpNaN;
			continue;
		}
		float dist = edge->oppDist();
		if (OpMath::IsNaN(dist)) {
			priorDist = lastDist = OpNaN;
			continue;
		}
		if (dist * lastDist < 0) {
			OP_ASSERT(ref.limits.size());  // confirm that intersection was found
			return true;  // don't look further if crossing is detected
		}
		if (fabsf(dist) <= OpEpsilon) {
			OP_ASSERT(ref.limits.size());  // confirm that intersection was found
			return true;  // don't look further if crossing is detected
		}
		if (fabsf(priorDist) < fabsf(lastDist) && fabsf(lastDist) < fabsf(dist)) {
			// verify that edge does not overlap a found sect
			for (auto limit : ref.limits) {
				if (edge->start.t <= limit.seg.t && limit.seg.t <= edge->end.t)
					OP_ASSERT(0);
			}
		}
		priorDist = lastDist;
		lastDist = dist;
	}

#if 0
		if (!key || dist * lastDist < 0) { // signs differ
			if (key)
				SetupDebugImage();
		#if OP_DEBUG_IMAGE
			resetFocus();
		#endif
			if (key) OpDebugOut("key:" + OpDebugIntToHex(key)
					+ " testDepth:" + STR(testDepth)
					+ " depth:" + STR(cc.depth) + "\n");
			if (5 <= cc.depth)
				OpDebugOut("");
			return true;
		}
		lastDist = dist;
#endif

	return false;
}
#endif

#if 0  // !!! out of date
static void testCc(uint32_t key, OpSegment* seg, OpSegment* opp) {
	SetupDebugImage();
	OpCurveCurve reference(seg, opp);
	SectFound refResult = reference.divideAndConquer();
	if (SectFound::fail == refResult) {
		OP_ASSERT(reference.edgeCurves.c.size() >= 4);  // if less, assert to understand why
		OP_ASSERT(reference.oppCurves.c.size() >= 4);  // if less, assert to understand why
		// compute all oppEnd values
		auto checkDist = [](CcCurves& curves) {
			// verify that edges in oppEnd values imply opp dists that diverge from zero 
			float lastDist = curves.c[0]->oppDist();
			for (size_t index = 1; index < curves.c.size(); ++index) {
				float dist = curves.c[index]->oppDist();
				OP_ASSERT(!lastDist || lastDist * dist > 0);
				OP_ASSERT(!lastDist || fabsf(lastDist) < fabsf(dist));
				lastDist = dist;
			}
		};
		reference.edgeCurves.endDist(seg, opp);
		checkDist(reference.edgeCurves);
		reference.oppCurves.endDist(opp, seg);
		checkDist(reference.oppCurves);
	}
    if (SectFound::add == refResult || reference.limits.size())
        reference.findUnsectable();
    OpCurveCurve cc(seg, opp);
	int testDepth = 0;
	for (cc.depth = 1; cc.depth < cc.maxDepth; ++cc.depth) {
		cc.edgeCurves.endDist(cc.seg, cc.opp);
		cc.oppCurves.endDist(cc.opp, cc.seg);
		if (!cc.setOverlaps()) {
			OP_ASSERT(0);   // !!! I want to see this happen...
			continue;
		}
		int edgeOverlaps = cc.edgeCurves.overlaps();
		int oppOverlaps = cc.oppCurves.overlaps();
		if (!edgeOverlaps || !oppOverlaps)
			return;
		if (2 < edgeOverlaps) {
			++testDepth;
			if (checkNormals(key, testDepth, cc, reference))
				break;
		}
		if (edgeOverlaps > maxOverlaps || oppOverlaps > maxOverlaps)
			return;
		CcCurves eSplits, oSplits;
		cc.splitHulls(CurveRef::edge, eSplits);
		cc.splitHulls(CurveRef::opp, oSplits);
		cc.edgeCurves.c.swap(eSplits.c);
		cc.oppCurves.c.swap(oSplits.c);
	}
}
#endif

void CCTest() {
#if 0 // !!! out of date
	struct IntPt {
		int x;
		int y;
	} p1[3], p2[3];
	SkPath ska, skb;
	OpInPath ia(&ska), ib(&skb);
#define FIRST_A_VALUE 1  // set to zero for full test
	for (int a = FIRST_A_VALUE; a < 16; ++a) {
		p1[0] = { a & 0x03, a >> 2 };
        for (int b = a ; b < 16; ++b) {
			p1[1] = { b & 0x03, b >> 2 };
            for (int c = b ; c < 16; ++c) {
				p1[2] = { c & 0x03, c >> 2 };
				OpPoint f1[3] = { { (float) p1[0].x, (float) p1[0].y }, 
								  { (float) p1[1].x, (float) p1[1].y }, 
							      { (float) p1[2].x, (float) p1[2].y } };
				for (int e = 0 ; e < 16; ++e) {
					for (int f = e ; f < 16; ++f) {
						p2[0] = { f & 0x03, f >> 2 };
						for (int g = f ; g < 16; ++g) {
							p2[1] = { g & 0x03, g >> 2 };
							for (int h = g ; h < 16; ++h) {
								p2[2] = { h & 0x03, h >> 2 };
								OpPoint f2[3] = { { (float) p2[0].x, (float) p2[0].y }, 
												  { (float) p2[1].x, (float) p2[1].y }, 
												  { (float) p2[2].x, (float) p2[2].y } };
								if (f1[0] == f1[2] || f2[0] == f2[2])
									continue;
							OpContours contours(ia, ib, OpOperator::Intersect);
						#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
							debugGlobalContours = &contours;
						#endif
							OpContour* head = contours.makeContour(OpOperand::left);
								if (!head->addQuad(f1))
									continue;
								size_t qCount = head->segments.size();
								if (!head->addQuad(f2))
									continue;
								for (size_t cq = 0; cq < qCount; ++cq) {
									if (OpType::quad != head->segments[cq].c.type)
										continue;
									for (size_t cr = qCount; cr < head->segments.size(); ++cr) {
										if (OpType::quad != head->segments[cr].c.type)
											continue;
										OP_ASSERT(cq < 4 && cr < 4);
										uint32_t key = a | (b << 4) | (c << 8)
												| (f << 12) | (g << 16) | (h << 20) 
												| (cq << 24) | (cr << 26);
										testCc(key, &head->segments[cq], &head->segments[cr]);
									}
								}
							}
						}
					}
                }
            }
        }
    }
#endif
}

void CCTestKey(uint32_t key) {
#if 0 // !!! out of date
	struct IntPt {
		int x;
		int y;
	} p1[3], p2[3];
	SkPath ska, skb;
	OpInPath ia(&ska), ib(&skb);
	int a = key & 0xF;
	p1[0] = { a & 0x03, a >> 2 };
	int b = (key >> 4) & 0xF;
	p1[1] = { b & 0x03, b >> 2 };
	int c = (key >> 8) & 0xF;
	p1[2] = { c & 0x03, c >> 2 };
	OpPoint f1[3] = { { (float) p1[0].x, (float) p1[0].y }, 
						{ (float) p1[1].x, (float) p1[1].y }, 
						{ (float) p1[2].x, (float) p1[2].y } };
	int f = (key >> 12) & 0xF;
	p2[0] = { f & 0x03, f >> 2 };
	int g = (key >> 16) & 0xF;
	p2[1] = { g & 0x03, g >> 2 };
	int h = (key >> 20) & 0xF;
	p2[2] = { h & 0x03, h >> 2 };
	OpPoint f2[3] = { { (float) p2[0].x, (float) p2[0].y }, 
						{ (float) p2[1].x, (float) p2[1].y }, 
						{ (float) p2[2].x, (float) p2[2].y } };
	OP_ASSERT(f1[0] != f1[2] && f2[0] != f2[2]);
	OpContours contours(ia, ib, OpOperator::Intersect);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
	debugGlobalContours = &contours;
#endif
	OpContour* head = contours.makeContour(OpOperand::left);
	OP_EXECUTE_AND_ASSERT(head->addQuad(f1));
	OP_EXECUTE_AND_ASSERT(head->addQuad(f2));
	size_t cq = (key >> 24) & 0x3;
	OP_ASSERT(OpType::quad == head->segments[cq].c.type);
	size_t cr = (key >> 26) & 0x3;
	OP_ASSERT(OpType::quad == head->segments[cr].c.type);
	testCc(0, &head->segments[cq], &head->segments[cr]);
#endif
}

#endif
