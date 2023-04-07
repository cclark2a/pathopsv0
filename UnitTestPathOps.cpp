//       1         2         3         4         5         6         7         8         9         0
//34567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#include "OpDebug.h"
#if OP_RELEASE_TEST

// math test
#include <random>
#include <string>
#include "OpMath.h"
#include "OpContour.h"
#include "src/pathops/SkPathOpsConic.h"
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"

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
	rootCellar allGood1 = { 0, .1f, .5f};
	rootCellar allGood2 = { .7f, .9f, 1 };
	rootCellar allBad1 = { 0 - OpEpsilon, 1 + OpEpsilon, OpInfinity };
	rootCellar allBad2 = { -OpInfinity, OpNaN };
	rootCellar someOfEach1 = { 0, .1f, 1 + OpEpsilon };
	rootCellar someOfEach2 = { .5f, OpInfinity };
	int good = OpMath::KeepValidTs(allGood1, 3);
	ASSERT_I(allGood1.size() == good, good);
	good = OpMath::KeepValidTs(allGood2, 3);
	ASSERT_I(allGood2.size() == good, good);
	int bad = OpMath::KeepValidTs(allBad1, 3);
	ASSERT_I(0 == bad, bad);
	bad = OpMath::KeepValidTs(allBad2, 2);
	ASSERT_I(0 == bad, bad);
	int mix = OpMath::KeepValidTs(someOfEach1, 3);
	ASSERT_I(2 == mix, mix);
	mix = OpMath::KeepValidTs(someOfEach2, 2);
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
		rootCellar tVals;
		double tValsD[2];
		int sk_result = SkDCubic::RootsReal(p[0], p[1], p[2], p[3], tValsD);
		int result = OpMath::CubicRootsReal(p[0], p[1], p[2], p[3], tVals);
		ASSERT(sk_result == result);
		for (int r = 0; r < result; ++r) {
			ASSERT(tVals[r] == tValsD[r]);
		}
	}

	// test CubicRootsValidT
	for (unsigned index = 0; index < ARRAY_COUNT(p) - 3; ++index) {
		rootCellar tVals;
		double tValsD[2];
		int sk_result = SkDCubic::RootsValidT(p[0], p[1], p[2], p[3], tValsD);
		int result = OpMath::CubicRootsValidT(p[0], p[1], p[2], p[3], tVals);
		ASSERT(sk_result == result);
		for (int r = 0; r < result; ++r) {
			ASSERT(tVals[r] == tValsD[r]);
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
		rootCellar tVals;
		double tValsD[2];
		int sk_result = SkDQuad::RootsReal(p[0], p[1], p[2], tValsD);
		int result = OpMath::QuadRootsReal(p[0], p[1], p[2], tVals);
		ASSERT(sk_result == result);
		for (int r = 0; r < result; ++r) {
			ASSERT(tVals[r] == tValsD[r]);
		}
	}

	// test QuadRootsValidT
	for (unsigned index = 0; index < ARRAY_COUNT(p); ++index) {
		rootCellar tVals;
		double tValsD[2];
		int sk_result = SkDQuad::RootsValidT(p[0], p[1], p[2], tValsD);
		int result = OpMath::QuadRootsValidT(p[0], p[1], p[2], tVals);
		ASSERT(sk_result == result);
		for (int r = 0; r < result; ++r) {
			ASSERT(tVals[r] == tValsD[r]);
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
					assert((size_t) roots + (size_t) edgeRoots <= edgeCepts.size());
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

void OpCurveTest() {
	const OpPoint pts[] = { { 100, 100 }, { 200, 100 }, { 200, 200 }, {100, 200} };
	OpCurve curve = { pts, pointType };
	OpLine line = { pts };
	OpQuad quad = { pts };
	OpConic conic = { pts, .5 };
	OpCubic cubic = { pts };

	// test pointCount
	ASSERT(1 == curve.pointCount());
	ASSERT(2 == line.pointCount());
	ASSERT(3 == quad.pointCount());
	ASSERT(3 == conic.pointCount());
	ASSERT(4 == cubic.pointCount());

	// test OpCurve toVertical
	const std::array<OpPoint, 2> iLinePts = {{ {1, 1}, {4, 4} }};
	const OpPoint iQPts[] = {  { 2, 4}, {1, 3}, {3, 1} };
	OpQuad iQuad = { iQPts };
	OpCurve rotated;
	iQuad.toVertical(iLinePts, rotated);
	ASSERT(6 == rotated.pts[0].x);
	ASSERT(6 == rotated.pts[1].x);
	ASSERT(-6 == rotated.pts[2].x);
	OP_DEBUG_CODE(curve.dump());
}

void OpLineTest() {
	// test OpLine axisRayHit
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	OpLine iLine = { iLinePts[0], iLinePts[1] };
	rootCellar intercepts;
	int result = iLine.axisRayHit(Axis::vertical, 2.5, intercepts);
	ASSERT(1 == result);
	ASSERT(.5 == intercepts[0]);
	result = iLine.axisRayHit(Axis::horizontal, 2.5, intercepts);
	ASSERT(1 == result);
	ASSERT(.5 == intercepts[0]);


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
	const std::array<OpPoint, 2> vert = {{ { 2.5, 2 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 7, 2.5 } }};
	result = l1.rayIntersect(vert, intercepts);
	ASSERT(1 == result);
	ASSERT(.75 == intercepts[0]);
	result = l1.rayIntersect(horz, intercepts);
	ASSERT(1 == result);
	ASSERT(.25 == intercepts[0]);
	result = l1.rayIntersect(iLinePts, intercepts);
	ASSERT(1 == result);
	ASSERT(.5 == intercepts[0]);

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
	rootCellar intercepts;
	int result = q.axisRayHit(Axis::vertical, 150, intercepts);
	ASSERT(1 == result);
	OpPoint pt = q.ptAtT(intercepts[0]);
	ASSERT(150 == pt.x);

	result = q.axisRayHit(Axis::horizontal, 150, intercepts);
	ASSERT(1 == result);
	pt = q.ptAtT(intercepts[0]);
	ASSERT(150 == pt.y);

	// test OpQuad coefficients
	// !!! tested by quad axisRayHit

	OpPoint qe1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 } };
	OpPoint qe2[] = { { 1, 1 }, { 2, 2 }, { 3, 1 } };
	// test OpQuad extrema
	OpQuad qe = { qe1 };
	rootCellar extrema;
	result = qe.extrema(XyChoice::inX, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);
	result = qe.extrema(XyChoice::inY, extrema);
	ASSERT(0 == result);
	qe = { qe2 };
	result = qe.extrema(XyChoice::inX, extrema);
	ASSERT(0 == result);
	result = qe.extrema(XyChoice::inY, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);

	// test OpQuad rayIntersect
	rootCellar icepts[3];
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	OpQuad qi1 = { &iQPts[0] };
	const std::array<OpPoint,2> vert = {{ { 2.5, 2 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 7, 2.5 } }};
	result = qi1.rayIntersect(vert, icepts[0]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.86f, icepts[0][0], 0.8604f));
	result = qi1.rayIntersect(horz, icepts[1]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.5811f, icepts[1][0], 0.5812f));
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	result = qi1.rayIntersect(iLinePts, icepts[2]);
	ASSERT(1 == result);
	// answer is sqrt(2)/2
	ASSERT(OpMath::Between(0.7071f, icepts[2][0], 0.7072f));

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
	std::array<OpPoint, 4> qdst;
	OpPtT icPtT[2] = {{ qi1.ptAtT(icepts[0][0]), icepts[0][0] }, 
		{ qi1.ptAtT(icepts[1][0]), icepts[1][0] }};
	qi1.subDivide(icPtT[0], icPtT[1], qdst);
	pt = qi1.ptAtT(icepts[0][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - qdst[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - qdst[0].y, 0.000001f));
	pt = qi1.ptAtT(icepts[1][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - qdst[2].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - qdst[2].y, 0.000001f));

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
	rootCellar intercepts;
	const OpPoint q1[] = { { 100, 100 }, { 200, 100 }, { 200, 200 } };
	OpConic c = { q1, sqrtf(2)/2 };
	int result = c.axisRayHit(Axis::vertical, 150, intercepts);
	ASSERT(1 == result);
	OpPoint pt = c.ptAtT(intercepts[0]);
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
	rootCellar extrema;
	result = ce.extrema(XyChoice::inX, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);
	result = ce.extrema(XyChoice::inY, extrema);
	ASSERT(0 == result);
	ce = { ce2, sqrtf(2) / 2 };
	result = ce.extrema(XyChoice::inX, extrema);
	ASSERT(0 == result);
	result = ce.extrema(XyChoice::inY, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);

	// test OpConic rayIntersect
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	OpConic ci1 = { &iQPts[0], sqrtf(2) / 2 };
	const std::array<OpPoint,2> vert = {{ { 2.5, 2 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 7, 2.5 } }};
	rootCellar icepts[3];
	result = ci1.rayIntersect(vert, icepts[0]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.817f, icepts[0][0], 0.818f));
	result = ci1.rayIntersect(horz, icepts[1]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.5581f, icepts[1][0], 0.5582f));
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	result = ci1.rayIntersect(iLinePts, icepts[2]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.6589f, icepts[2][0], 0.659f));

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
	std::array<OpPoint, 4> cdst;
	OpPtT icPtT[2] = { { ci1.ptAtT(icepts[0][0]), icepts[0][0] },
		{ ci1.ptAtT(icepts[1][0]), icepts[1][0] } };
	float cW;
	ci1.subDivide(icPtT[0], icPtT[1], cdst, &cW);
	pt = ci1.ptAtT(icepts[0][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst[0].y, 0.000001f));
	pt = ci1.ptAtT(icepts[1][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst[2].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst[2].y, 0.000001f));

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
	rootCellar intercepts;
	int result = c.axisRayHit(Axis::vertical, 150, intercepts);
	ASSERT(1 == result);
	OpPoint pt = c.ptAtT(intercepts[0]);
	ASSERT(OpMath::Between(-.0001f, pt.x - 150, .0001f));

	result = c.axisRayHit(Axis::horizontal, 150, intercepts);
	ASSERT(1 == result);
	pt = c.ptAtT(intercepts[0]);
	ASSERT(OpMath::Between(-.0001f, pt.y - 150, .0001f));

	// test OpCubic coefficients
	// !!! tested by axisRayHit

	// test OpCubic extrema
	OpPoint ce1[] = { { 1, 1 }, { 2, 2 }, { 2, 3 }, { 1, 4 } };
	OpPoint ce2[] = { { 1, 1 }, { 2, 2 }, { 3, 2 }, { 4, 1 } };
	OpCubic ce = { ce1 };
	rootCellar extrema;
	result = ce.extrema(XyChoice::inX, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);
	result = ce.extrema(XyChoice::inY, extrema);
	ASSERT(0 == result);
	ce = { ce2 };
	result = ce.extrema(XyChoice::inX, extrema);
	ASSERT(0 == result);
	result = ce.extrema(XyChoice::inY, extrema);
	ASSERT(1 == result);
	ASSERT(0.5 == extrema[0]);

	// test OpCubic inflections
	OpPoint ci1[] = { { 1, 1 }, { 2, 2 }, { 1, 3 }, { 2, 4 } };
	OpCubic ci = { ci1 };
	rootCellar inflections;
	result = ci.inflections(inflections);
	ASSERT(1 == result);
	ASSERT(0.5 == inflections[0]);

	// test OpCubic interp
	// !!! tested by subdivide

	// test OpCubic rayIntersect
	rootCellar icepts[3];
	const OpPoint iCPts[] = { { 2, 4 }, { 1, 3 }, { 1, 2 }, { 3, 1 } };
	ci = { &iCPts[0] };
	const std::array<OpPoint, 2> vert = {{ { 2.5, 2 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 7, 2.5 } }};
	result = ci.rayIntersect(vert, icepts[0]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.9084f, icepts[0][0], 0.9085f));
	pt = ci.ptAtT(icepts[0][0]);
	ASSERT(OpMath::Between(-.0001f, pt.x - 2.5, .0001f));
	result = ci.rayIntersect(horz, icepts[1]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.5f, icepts[1][0], 0.5001f));
	pt = ci.ptAtT(icepts[1][0]);
	ASSERT(OpMath::Between(-.0001f, pt.y - 2.5, .0001f));
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	result = ci.rayIntersect(iLinePts, icepts[2]);
	ASSERT(1 == result);
	ASSERT(OpMath::Between(0.7320f, icepts[2][0], 0.7321f));
	pt = ci.ptAtT(icepts[2][0]);
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
	std::array<OpPoint, 4> cdst;
	OpPtT icPtT[2] = { { ci.ptAtT(icepts[0][0]), icepts[0][0] },
		{ ci.ptAtT(icepts[1][0]), icepts[1][0] } };
	ci.subDivide(icPtT[0], icPtT[1], cdst);
	pt = ci.ptAtT(icepts[0][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst[0].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst[0].y, 0.000001f));
	pt = ci.ptAtT(icepts[1][0]);
	ASSERT(OpMath::Between(-0.000001f, pt.x - cdst[3].x, 0.000001f));
	ASSERT(OpMath::Between(-0.000001f, pt.y - cdst[3].y, 0.000001f));

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

#include "OpEdgeBuilder.h"
#include "OpEdges.h"
#include "PathOps.h"

bool OpPathOpsTest1(const SkPath& one, const SkPath& two, SkPath* result) {
	OpInPath op1(&one);
	OpInPath op2(&two);
	OpOutPath opOut(result);
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::Intersect, opOut);
	assert(didIt);
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
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::ExclusiveOr, opOut);
	assert(didIt);
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
			assert(result.getBounds() == r3);
			result.reset();
			(void)OpPathOpsTest1(two, one, &result);
			assert(result.getBounds() == r3);
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
	assert(result.getBounds() == r1);
}

void OpTestOpEdgesConcidenceCheck() {
	for (float a = 2; a < 10; a += 2) {
		for (float b = 2; b < 10; b += 2) {
			if (a == b)
				continue;
			for (float c = 2; c < 10; c += 2) {
				for (float d = 2; d < 10; d += 2) {
					if (c == d)
						continue;
					OpPoint ab[2] = { { a, 8 }, { b, 8 } };
					OpPoint cd[2] = { { c, 8 }, { d, 8 } };
					OpContours contours(OpOperator::Intersect);
					contours.left = OpFillType::evenOdd;
					OpContour contour(&contours, OpOperand::left);
					OpSegment seg(ab, lineType, &contour);
					OpSegment oSeg(cd, lineType, &contour);
					OpEdge edge(&seg, { ab[0], 0}, { ab[1], 1 }  OP_DEBUG_PARAMS(EdgeMaker::opTest));
					OpEdge oEdge(&oSeg, { cd[0], 0 }, { cd[1], 1 }  OP_DEBUG_PARAMS(EdgeMaker::opTest));
					OpEdges::CoincidentCheck(oEdge, edge);
					float _ab[2] = { a, b};
					float _cd[2] = { c, d};
					if (_ab[0] > _ab[1]) std::swap(_ab[0], _ab[1]);
					if (_cd[0] > _cd[1]) std::swap(_cd[0], _cd[1]);
					float min = std::max(_ab[0], _cd[0]);
					float max = std::min(_ab[1], _cd[1]);
					if (min > max) {
						assert(!seg.intersections.size());
						assert(!oSeg.intersections.size());
						continue;
					}
					if (min == max) {
						assert(1 == seg.intersections.size());
						assert(seg.intersections[0]->ptT.t == 0 || seg.intersections[0]->ptT.t == 1);
						assert(1 == oSeg.intersections.size());
						assert(oSeg.intersections[0]->ptT.t == 0 || oSeg.intersections[0]->ptT.t == 1);
					} else {
						assert(2 == seg.intersections.size());
						assert(2 == oSeg.intersections.size());
					}
					OP_DEBUG_CODE(seg.debugValidate());
					OP_DEBUG_CODE(oSeg.debugValidate());
				}
			}
		}
	}
}

struct WindStateTest {
	int wind;
	int sum;
	WindState state;
};

void OpTest_WindState() {
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
			assert(0);
	}
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
		assert(test.state == edge2.windZero(OpOperand::left));
	}
}
#endif

// bits 0: left winding  1: right windng  2: left sum  3: right sum
bool setWinding(OpEdge& edge, int index) {
	if (0 == (index & 3))
		return false;	// don't test cases with no flips
	edge.winding.left = (index & 1) ? 1 : 0;
	edge.winding.right = (index & 2) ? 1 : 0;
	edge.sum.left = (index & 4) ? 1 : 0;
	edge.sum.right = (index & 8) ? 1 : 0;
	// return false if there are different zero sides
	if (3 == (index & 3) && edge.sum.left != edge.sum.right)
		return false;
	return true;
}

void OpTest_EdgeZero() {
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
					seg1.addIntersection(OpPtT(data1[0][0], 0)
							OP_DEBUG_PARAMS(IntersectMaker::opTestEdgeZero1));
					seg1.addIntersection(OpPtT(data1[0][1], 1)
							OP_DEBUG_PARAMS(IntersectMaker::opTestEdgeZero2));
					seg1.makeEdges();
					OpEdge& edge1 = seg1.edges[0];
					if (!setWinding(edge1, w1))
						continue;
					OpSegment& seg2 = *std::next(head->segments.begin());
					seg2.addIntersection(OpPtT(data2[0][0], 0)
							OP_DEBUG_PARAMS(IntersectMaker::opTestEdgeZero3));
					seg2.addIntersection(OpPtT(data2[0][1], 1)
							OP_DEBUG_PARAMS(IntersectMaker::opTestEdgeZero4));
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
}

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
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::Intersect, &result);
	assert(didIt);
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
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::Intersect, opOut);
	assert(didIt);
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
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::Intersect, opOut);
	assert(didIt);
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
	OP_DEBUG_CODE(bool didIt =) PathOps(op1, op2, OpOperator::Intersect, opOut);
	assert(didIt);
	result.dump();
	OpDebugOut("");
}

extern void run_all_tests();

#if OP_DEBUG
void onetest() {
	OpPoint data[] = { 
		  { OpDebugBitsToFloat(0x42f7639d), OpDebugBitsToFloat(0x4116d2c4) }  // {123.694557, 9.426456}
		, { OpDebugBitsToFloat(0x42f7639d), OpDebugBitsToFloat(0x4116d2c3) }  // {123.694557, 9.426455}
		, { OpDebugBitsToFloat(0x42f7639b), OpDebugBitsToFloat(0x4116d2c4) }  // {123.694542, 9.426456}
		, { OpDebugBitsToFloat(0x42f7639b), OpDebugBitsToFloat(0x4116d2c3) }  // {123.694542, 9.426455}
	}; 
	OpCurve curve(data, cubicType);
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

void OpTest(bool terminateEarly) {
#ifdef NDEBUG
	OpCubicErrorTest(CubicTest::ptAtT, SpeedTest::on);
	OpCubicErrorTest(CubicTest::edgeRay, SpeedTest::on);
	OpCubicErrorTest(CubicTest::axisRay, SpeedTest::on);
	OpCubicErrorTest(CubicTest::doubleRay, SpeedTest::on);
	if (terminateEarly)
		exit(0);
#endif
	OpTest_WindState();
//	OpTest_WindZero();
	OpTest_EdgeZero();
//	OpPoint pt = testMinMax({ (float) rand(), (float)rand() }, { (float)rand(), (float)rand() });
//	OpDebugOut(std::to_string(pt.x));
	OP_DEBUG_CODE(onetest());
	run_all_tests();
	OpTestQuadCoin2();
	OpTestQuadCoin();
	OpTestQuadQuad();
	OpMathTest();
	OpCurveTest();
	OpLineTest();
	OpQuadTest();
	OpConicTest();
	OpCubicTest();
	OpTestOpEdgesConcidenceCheck();
	OpTestQuadLine();
	OpTestCoincidence();
	OpTestXor();
	OpTestRect();
}

#include "include/core/SkCanvas.h"

static void drawPath(SkCanvas* canvas, const SkPath& path) {
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	canvas->drawPath(path, paint);
}

void OpQuadDraw(SkCanvas* canvas) {
	const std::array<OpPoint,2> vert = {{ { 2.5, 1 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 1, 2.5 } }};
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};

	// draw quadratic
	OpQuad qi1 = { iQPts };
	rootCellar intercepts[3];
	int results[3];
	results[0] = qi1.rayIntersect(vert, intercepts[0]);
	results[1] = qi1.rayIntersect(horz, intercepts[1]);
	results[2] = qi1.rayIntersect(iLinePts, intercepts[2]);
	SkPath path;
	path.moveTo(vert[0].x, vert[0].y);
	path.lineTo(vert[1].x, vert[1].y);
	path.moveTo(horz[0].x, horz[0].y);
	path.lineTo(horz[1].x, horz[1].y);
	path.moveTo(iLinePts[0].x, iLinePts[0].y);
	path.lineTo(iLinePts[1].x, iLinePts[1].y);
	path.moveTo(iQPts[0].x, iQPts[0].y);
	path.quadTo(iQPts[1].x, iQPts[1].y, iQPts[2].x, iQPts[2].y);
	drawPath(canvas, path);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	for (int x = 0; x < 3; ++x) {
		OpPoint pt = qi1.ptAtT(intercepts[x][0]);
		canvas->drawCircle(pt.x, pt.y, 0.2f, paint);
	}
	for (float fx = 0; fx <= 1; fx += .5) {
		OpVector norm = qi1.normal(fx);
		OpVector tan = qi1.tangent(fx);
		OpPoint pt = qi1.ptAtT(fx);
		paint.setColor(SK_ColorRED);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + norm.dx, pt.y + norm.dy }, paint);
		paint.setColor(SK_ColorBLUE);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + tan.dx, pt.y + tan.dy }, paint);
	}
}

void OpConicDraw(SkCanvas* canvas) {
	const std::array<OpPoint, 2> vert = {{ { 2.5, 1 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 1, 2.5 } }};
	const OpPoint iQPts[] = { { 2, 4 }, { 1, 3 }, { 3, 1 } };
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	OpConic ci1 = { &iQPts[0], sqrtf(2) / 2 };
	rootCellar intercepts[3];
	int results[3];
	results[0] = ci1.axisRayHit(Axis::vertical, 2.5, intercepts[0]);
	results[1] = ci1.rayIntersect(horz, intercepts[1]);
	results[2] = ci1.rayIntersect(iLinePts, intercepts[2]);
	SkPath path;
	path.moveTo(vert[0].x, vert[0].y);
	path.lineTo(vert[1].x, vert[1].y);
	path.moveTo(horz[0].x, horz[0].y);
	path.lineTo(horz[1].x, horz[1].y);
	path.moveTo(iLinePts[0].x, iLinePts[0].y);
	path.lineTo(iLinePts[1].x, iLinePts[1].y);
	path.moveTo(iQPts[0].x, iQPts[0].y);
	path.conicTo(iQPts[1].x, iQPts[1].y, iQPts[2].x, iQPts[2].y, ci1.weight);
	drawPath(canvas, path);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	for (int x = 0; x < 3; ++x) {
		for (int y = 0; y < results[x]; ++y) {
			OpPoint pt = ci1.ptAtT(intercepts[x][y]);
			canvas->drawCircle(pt.x, pt.y, 0.2f, paint);
		}
	}
	for (float fx = 0; fx <= 1; fx += .5) {
		OpVector norm = ci1.normal(fx);
		OpVector tan = ci1.tangent(fx);
		OpPoint pt = ci1.ptAtT(fx);
		paint.setColor(SK_ColorRED);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + norm.dx, pt.y + norm.dy }, paint);
		paint.setColor(SK_ColorBLUE);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + tan.dx, pt.y + tan.dy }, paint);
	}
}

void OpCubicDraw(SkCanvas* canvas) {
	const OpPoint iCPts[] = { { 2, 4 }, { 1, 3 }, { 1, 2 }, { 3, 1 } };
	const std::array<OpPoint, 2> iLinePts = {{ { 1, 1 }, { 4, 4 } }};
	const std::array<OpPoint, 2> vert = {{ { 2.5, 1 }, { 2.5, 3 } }};
	const std::array<OpPoint, 2> horz = {{ { 4, 2.5 }, { 1, 2.5 } }};
	OpCubic ci1 = { iCPts };
	rootCellar intercepts[3];
	int results[3];
	results[0] = ci1.rayIntersect(vert, intercepts[0]);
	results[1] = ci1.rayIntersect(horz, intercepts[1]);
	results[2] = ci1.rayIntersect(iLinePts, intercepts[2]);
	SkPath path;
	path.moveTo(iCPts[0].x, iCPts[0].y);
	path.cubicTo(iCPts[1].x, iCPts[1].y, iCPts[2].x, iCPts[2].y,
		iCPts[3].x, iCPts[3].y);
	path.moveTo(vert[0].x, vert[0].y);
	path.lineTo(vert[1].x, vert[1].y);
	path.moveTo(horz[0].x, horz[0].y);
	path.lineTo(horz[1].x, horz[1].y);
	path.moveTo(iLinePts[0].x, iLinePts[0].y);
	path.lineTo(iLinePts[1].x, iLinePts[1].y);
	drawPath(canvas, path);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	for (int x = 0; x < 3; ++x) {
		OpPoint pt = ci1.ptAtT(intercepts[x][0]);
		canvas->drawCircle(pt.x, pt.y, 0.2f, paint);
	}
	for (float fx = 0; fx <= 1; fx += .5) {
		OpVector norm = ci1.normal(fx);
		OpVector tan = ci1.tangent(fx);
		OpPoint pt = ci1.ptAtT(fx);
		paint.setColor(SK_ColorRED);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + norm.dx, pt.y + norm.dy }, paint);
		paint.setColor(SK_ColorBLUE);
		canvas->drawLine({ pt.x, pt.y }, { pt.x + tan.dx, pt.y + tan.dy }, paint);
	}
}

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
	OpTightBounds bounds(cubic);
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
#if 0
	OpPoint data[] = {
		{ OpDebugBitsToFloat(0x402f3d1c), OpDebugBitsToFloat(0x40852e3e) }, // {2.7381, 4.16189}
		{ OpDebugBitsToFloat(0x402be4f9), OpDebugBitsToFloat(0x40852e3e) }, // {2.68585, 4.16189}
		{ OpDebugBitsToFloat(0x4029b581), OpDebugBitsToFloat(0x40840f04) }, // {2.6517, 4.12683}
		{ OpDebugBitsToFloat(0x4028aea9), OpDebugBitsToFloat(0x4082171f) }  // {2.63566, 4.06532}
	};  // type:cubic t:  0.112702, 0.207849
#endif
}

#endif
