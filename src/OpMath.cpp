// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpMath.h"
#include <cmath>

// if t is nearly end of range, make it end of range
// motivation for this is test cubics_d, which generates yExtrema very nearly equal to 1.
// 'interior' is only used for extrema and inflections
OpRoots OpRoots::keepInteriorTs(float start, float end) {
	(void) keepValidTs(start, end);
	OpRoots interior;
	for (float tValue : roots) {
		if (start >= tValue - OpEpsilon || tValue + OpEpsilon >= end)
			continue;
		interior.roots.push_back(tValue);
	}
	std::swap(interior.roots, roots);
	return *this;
}

OpRoots OpRoots::keepValidTs(float start, float end) {
	OpRoots validTs;
	for (float tValue : roots) {
		if (OpMath::IsNaN(tValue) || start > tValue || tValue > end)
			continue;
		if (tValue < start + OpEpsilon)
			tValue = start;
		else if (tValue > end - OpEpsilon)
			tValue = end;
		for (float alreadyFound : validTs.roots) {
			if (alreadyFound == tValue)
				goto notUnique;
		}
		validTs.roots.push_back(tValue);
	notUnique:
		;
	}
	std::swap(validTs.roots, roots);
	return *this;
}

#if 0
// move zero and one to the front so they get processed first
void OpRoots::prioritize01() {
	if (count <= 1)
		return;
	sort(); // zero, if present, is moved to front
	if (roots[count - 1] != 1)
		return; 
	if (roots[0] != 0) {
		std::swap(roots[0], roots[count - 1]);  // move 1 to front
		return;
	}
	if (count > 2)
		std::swap(roots[1], roots[count - 1]);  // zero in front, move 1 to second position
}
#endif

OpVector OpVector::normalize() {
	float len = length();
	if (!OpMath::IsFinite(len))
		return { 0, 0 };
	if (!len)
		return { OpNaN, OpNaN };
	float inverseLength = 1 / len;
	dx *= inverseLength;
	dy *= inverseLength;
	return *this;
}

bool OpVector::isFinite() const {
	return OpMath::IsFinite(dx) && OpMath::IsFinite(dy);
}

#if 0
// Use the axis with the greatest change to decide if mid is between start and end.
// While this doesn't work in general, the callers are dealing with coincident curves
// which may be close, but not on top of each other. When they are axis-aligned, the
// larger provides a better metric of whether multiple curves overlap.
bool OpPoint::Between(OpPoint start, OpPoint mid, OpPoint end) {
	OpVector scope = end - start;
	XyChoice xy = scope.larger();
	return OpMath::Between(start.choice(xy), mid.choice(xy), end.choice(xy));
}
#endif

bool OpPoint::isFinite() const {
	OP_ASSERT(!debugIsUninitialized());
	// return *this * 0 == OpPoint(0, 0);  // does not work with MSVC and /fp:fast
	return OpMath::IsFinite(x) && OpMath::IsFinite(y);
}

bool OpPoint::isNearly(OpPoint test, OpVector threshold) const {
	return OpMath::Equal(x, test.x, threshold.dx) && OpMath::Equal(y, test.y, threshold.dy);
}

void OpPoint::pin(const OpPoint a, const OpPoint b) {
	x = OpMath::PinUnsorted(a.x, x, b.x);
	y = OpMath::PinUnsorted(a.y, y, b.y);
}

void OpPoint::pin(const OpRect& r) {
	x = OpMath::PinSorted(r.left, x, r.right);
	y = OpMath::PinSorted(r.top, y, r.bottom);
}

#if 0
bool OpPoint::soClose(OpPoint test) const {
	return OpMath::Between(OpMath::CloseSmaller(x), test.x, OpMath::CloseLarger(x))
		&& OpMath::Between(OpMath::CloseSmaller(y), test.y, OpMath::CloseLarger(y));
}
#endif

OpPoint OpRect::center() const { 
	return { OpMath::Average(left, right), OpMath::Average(top, bottom) }; 
}

bool OpRect::isFinite() const {
	return OpMath::IsFinite(left) && OpMath::IsFinite(top)
		&& OpMath::IsFinite(right) && OpMath::IsFinite(bottom);
}

bool OpPtT::isNearly(const OpPtT& o, OpPoint threshold) const {
	return pt.isNearly(o.pt, threshold) || OpMath::EqualT(t, o.t);
}

bool OpMath::Equal(float a, float b, float threshold) {
	return (a < b ? a : b) + threshold >= (a < b ? b : a);
}

#if 0
// here's an example where the 32 bit float version doesn't work:
// cubic: {3, 4}, {2.8873, 4.1127}, {2.8, 4.16189}, {2.7381, 4.16189}
// (hex: {0x40400000, 0x40800000}, {0x4038c97e, 0x40839b42}, {0x40333334, 0x40852e3c}, {0x402f3d1c, 0x40852e3e})
// intersected with vertical line at: 2.86905241
// give roots: -1.53410935, 39953.0625, 5.97319698
// because Q3=5.57735578e+24  R2=5.57735520e+24
#define ACOS acosf
#define COS cosf
#define SQRT sqrtf
#define ONE 1.f
#define FABS fabsf
#define PI OpPI
#define CUBE_ROOT std::cbrtf
#else
#define ACOS acos
#define COS cos
#define SQRT sqrt
#define ONE 1.
#define FABS fabs
#define PI 3.1415926535897931
#define CUBE_ROOT std::cbrt
#endif

OpRoots OpMath::CubicRootsReal(OpCubicFloatType A, OpCubicFloatType B,
		OpCubicFloatType C, OpCubicFloatType D, MatchEnds common) {
	bool zeroIsRoot = MatchEnds::start == common || MatchEnds::both == common;
	bool oneIsRoot = MatchEnds::end == common || MatchEnds::both == common;
	if (0 == A)
		return QuadRootsDouble((float) B, (float) C, (float) D);
	// in thread_loops542, segment line 4 and segment cubic 2 intersect at one point: (0, 5)
	// line 4 points: {2.5, 2}, {0, 5}  
	// cubic 2 points: {0, 5}, {2.130306, 5}, {2.747878, 5}, {2.747878, 3.925804}
	// the cubics' values are:
	// A:4.7683715820312500e-07 B:13.614606857299805 C:-19.172750473022461 D:0.0000000000000000
	// if QuadRootsReal is called, a root with a value of 1 is found, and the op fails
	// QuadRootsDouble returns that same root as 1.4, and everything is OK
	// changed all three calls to double versions as a precaution
	if (zeroIsRoot || 0 == D) {  // 0 is one root
		OpRoots roots = QuadRootsDouble((float) A, (float) B, (float) C);
		roots.addEnd(0);
		return roots;
	}
	if (oneIsRoot || 0 == A + B + C + D) {  // 1 is one root
		OpRoots roots = QuadRootsDouble((float) A, (float) (A + B), (float) -D);
		roots.addEnd(1);
		return roots;
	}
	OpCubicFloatType invA = 1 / A;
	OpCubicFloatType a = B * invA;
	OpCubicFloatType b = C * invA;
	OpCubicFloatType c = D * invA;
	OpCubicFloatType a2 = a * a;
	OpCubicFloatType Q = (a2 - b * 3) / 9;
	OpCubicFloatType R = (2 * a2 * a - 9 * a * b + 27 * c) / 54;
	OpCubicFloatType R2 = R * R;
	OpCubicFloatType Q3 = Q * Q * Q;
	OpCubicFloatType R2MinusQ3 = R2 - Q3;
	OpCubicFloatType adiv3 = a / 3;
	OpCubicFloatType r;
	OpRoots roots;
	if (R2MinusQ3 < 0) {   // we have 3 real roots
		// the divide/root can, due to finite precisions, be slightly outside of -1...1
		OpCubicFloatType theta = ACOS(std::max(std::min(ONE, R / SQRT(Q3)), -ONE));
		OpCubicFloatType neg2RootQ = -2 * SQRT(Q);
		r = neg2RootQ * COS(theta / 3) - adiv3;
		roots.add((float) r);
		r = neg2RootQ * COS((theta + 2 * PI) / 3) - adiv3;
		roots.addEnd((float) r);
		r = neg2RootQ * COS((theta - 2 * PI) / 3) - adiv3;
		roots.addEnd((float) r);
	} else {  // we have 1 real root
		OpCubicFloatType sqrtR2MinusQ3 = SQRT(R2MinusQ3);
		// !!! need to rename this 'A' something else; since parameter is also 'A'
		A = FABS(R) + sqrtR2MinusQ3;
		A = CUBE_ROOT(A);
		if (R > 0)
			A = -A;
		if (A != 0)
			A += Q / A;
		r = A - adiv3;
		roots.add((float) r);
		if (R2 == Q3) {
			r = -A / 2 - adiv3;
			roots.addEnd((float) r);
		}
	}
	return roots;
}

// min, max not necessarily sorted
float OpMath::PinUnsorted(float min, float value, float max) {
	if (OpMath::IsNaN(min) || OpMath::IsNaN(max))
		return value;
	if (min > max)
		std::swap(min, max);
	return PinSorted(min, value, max);
}

float OpMath::PinSorted(float min, float value, float max) {
	OP_ASSERT(min <= max);
	return std::max(min, std::min(value, max));
}

OpPoint OpMath::Threshold(OpPoint pt1, OpPoint pt2) {
	auto threshold = [](float left, float right) {
		return std::max(1.f, std::max(fabsf(left), fabsf(right))) * OpEpsilon;
	};
	return OpPoint(threshold(pt1.x, pt2.x), threshold(pt1.y, pt2.y));
}

#if 0
bool LinePts::isPoint() const {
	return pts[1] == pts[0];
}

OpRoots LinePts::axisTanHit(Axis axis, float axisIntercept) const {
	const float* ptr = pts[0].asPtr(axis);
	if (fabsf(ptr[2] - ptr[0]) <= OpEpsilon)   // coincident line values are computed later
		return OpRoots(OpNaN, OpNaN);
	return OpRoots((axisIntercept - ptr[0]) / (ptr[2] - ptr[0]));
}

OpPoint LinePts::ptAtT(float t) const {
	if (0 == t)
		return pts[0];
	if (1 == t)
		return pts[1];
	return (1 - t) * pts[0] + t * pts[1];
}
#endif

bool LinePts::ptOnLine(OpPoint ctrlPt) const {
	if (!OpMath::Between(pts[0].x, ctrlPt.x, pts[1].x))
		return false;
	if (!OpMath::Between(pts[0].y, ctrlPt.y, pts[1].y))
		return false;
	OpVector sxy = ctrlPt - pts[0];
	OpVector dxy = pts[1] - pts[0];
	float nearStart = dxy.cross(sxy);
	if (fabsf(nearStart) > OpEpsilon)
		return false;
	OpVector exy = pts[1] - ctrlPt;
	float nearEnd = dxy.cross(exy);
	if (fabsf(nearEnd) > OpEpsilon)
		return false;
	return true;
}

#if 0
OpRoots LinePts::tangentIntersect(const LinePts& line) const {
	if (line.pts[0].x == line.pts[1].x)
		return axisTanHit(Axis::vertical, line.pts[0].x);
	if (line.pts[0].y == line.pts[1].y)
		return axisTanHit(Axis::horizontal, line.pts[0].y);
	float adj = line.pts[1].x - line.pts[0].x;
	float opp = line.pts[1].y - line.pts[0].y;
	LinePts rotated;
	for (int n = 0; n < 2; ++n) {
		OpVector v = pts[n] - line.pts[0];
		rotated.pts[n].x = v.dy * adj - v.dx * opp;
		rotated.pts[n].y = v.dy * opp + v.dx * adj;
	}
	return rotated.axisTanHit(Axis::vertical, 0);
}
#endif