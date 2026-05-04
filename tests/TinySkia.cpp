// (c) 2023, Cary Clark cclark2@gmail.com
#include <cstdarg>
#include <cmath>
#include "TinySkia.h"


#ifdef _MSC_VER
	#define COS_F(x) std::cosf(x)
	#define SIN_F(x) std::sinf(x)
	#define SQRT_F(x) std::sqrtf(x)
#else
	#define COS_F(x) cosf(x)
	#define SIN_F(x) sinf(x)
	#define SQRT_F(x) sqrtf(x)
#endif

#if OP_TEST_NEW_INTERFACE
// #include "PathOps.h"
#endif

OpPoint TinyCurve::lastPt() const {
	return pts[pointCount() - 1];
}

size_t TinyCurve::pointCount() const {
	switch(type) {
	case TinyType::line: return 2;
	case TinyType::quad: return 3;
	case TinyType::conic: return 3;
	case TinyType::cubic: return 4;
	default: OP_ASSERT(0); return 0;
	}
}

SkMatrix SkMatrix::MakeAll(float sx, float rx, float tx, float sy, float ry, float ty, 
		float p0, float p1, float p2) {
	SkMatrix m;
	m.m[0][0] = sx;
	m.m[0][1] = rx;
	m.m[0][2] = tx;
	m.m[1][0] = sy;
	m.m[1][1] = ry;
	m.m[1][2] = ty;
	return m;
}

void SkMatrix::mapPoints(OpPoint* pts, int count) const {
       for (int index = 0; index < count; ++index) {
               OpPoint pt = pts[index];
               pts[index].x = pt.x * m[0][0] + pt.y * m[0][1] + m[0][2];
               pts[index].y = pt.x * m[1][0] + pt.y * m[1][1] + m[1][2];
       }
}

void SkPath::reset() { 
	path.clear();
}

SkRect SkPath::getBounds() const {
	OpPointBounds b;
	for (const TinyCurve& c : path)
		for (size_t index = 0; index < c.pointCount(); ++index)
			OP_DEBUG_CODE(if (!c.pts[index].debugIsUninitialized()))
				b.add(c.pts[index]);
	SkRect bounds = { b.left, b.top, b.right, b.bottom };
	return bounds;
}

bool SkPath::isEmpty() const { 
	return !path.size(); 
}

bool SkPath::isFinite() const {
	for (const TinyCurve& c : path)
		for (size_t index = 0; index < c.pointCount(); ++index)
			if (OP_DEBUG_CODE(c.pts[index].debugIsUninitialized() ||) !c.pts[index].isFinite())
				return false;
	return true;
}

SkPath::RawIter::RawIter(SkPath const & p)
	: path(p)
	, index(0)
	, w(OpNaN)
	, nextClose(false)
	, nextMove(true) {
}

// !!! bogus: does not use new interface
SkPath::Verb SkPath::RawIter::next(SkPoint* pts) { 
	if (nextClose) {
		nextClose = false;
		nextMove = true;
		return SkPath::kClose_Verb;
	}
	if (index >= path.path.size())
		return SkPath::kDone_Verb; 
	const TinyCurve& c = path.path[index];
	if (nextMove) {
		nextMove = false;
		first = c.pts[0];
		pts[0] = { c.pts[0].x, c.pts[0].y };
		return SkPath::kMove_Verb;
	}
	if (first == c.lastPt())
		nextClose = true;
	for (size_t i = 0; i < c.pointCount(); ++i)
		pts[i] = { c.pts[i].x, c.pts[i].y };
	w = OpNaN;
	++index;
	switch (c.type) {
		case TinyType::line:
			return SkPath::kLine_Verb;
		case TinyType::quad:
			return SkPath::kQuad_Verb;
		case TinyType::conic:
			w = c.weight;
			return SkPath::kConic_Verb;
		case TinyType::cubic:
			return SkPath::kCubic_Verb;
		default:
			break;
	}
	OP_ASSERT(0);
	return SkPath::kDone_Verb;
}

void SkPath::moveTo(float x, float y) { 
	close();
	last = OpPoint(x, y);
	firstIndex = path.size();
}

void SkPath::lineTo(float x, float y) {
	TinyCurve curve { { last, OpPoint(x, y) }, 1, TinyType::line };
	path.push_back(std::move(curve));
	last = OpPoint(x, y);
}

void SkPath::quadTo(float x1, float y1, float x2, float y2) { 
	TinyCurve curve { { last, OpPoint(x1, y1), OpPoint(x2, y2) }, 1, TinyType::quad };
	path.push_back(std::move(curve));
	last = OpPoint(x2, y2);
}

void SkPath::conicTo(float x1, float y1, float x2, float y2, float w) {
	TinyCurve curve { { last, OpPoint(x1, y1), OpPoint(x2, y2) }, w, TinyType::conic };
	path.push_back(std::move(curve));
	last = OpPoint(x2, y2);
}

void SkPath::cubicTo(float x1, float y1, float x2, float y2, float x3, float y3) {
	TinyCurve curve { { last, OpPoint(x1, y1), OpPoint(x2, y2), OpPoint(x3, y3) }, 1, TinyType::cubic };
	path.push_back(std::move(curve));
	last = OpPoint(x3, y3);
}

void SkPath::close() {
	if (firstIndex >= path.size())
		return;
	OpPoint next = path[firstIndex].pts[0];
	if (last != next) {
		lineTo(next.x, next.y);
		last = next;
	}
}

void SkPath::arcTo(const SkRect& , float startAngle, float sweepAngle, bool forceMoveTo) {
	// intentionally unimplmented
}

void SkPath::addCircle(float x, float y, float r, SkPathDirection /* !!! ignored for now */) {  
	moveTo(x, y - r);
	conicTo(x + r, y - r, x + r, y, SQRT_F(2) / 2);
	conicTo(x + r, y + r, x, y + r, SQRT_F(2) / 2);
	conicTo(x - r, y + r, x - r, y, SQRT_F(2) / 2);
	conicTo(x - r, y - r, x, y - r, SQRT_F(2) / 2);
}

void SkPath::addPath(SkPath const& p) { 
	path.insert(path.end(), p.path.begin(), p.path.end());
}

void SkPath::addPath(SkPath const& p, const SkMatrix& m) {
	SkPath tmp;
	tmp.addPath(p);
	tmp.makeTransform(m);
	addPath(tmp);
}

void SkPath::addRect(float l, float t, float r, float b, SkPathDirection /* !!! ignored for now */) {
	moveTo(l, t);
	lineTo(r, t);
	lineTo(r, b);
	lineTo(l, b);
	lineTo(l, t);
}

void SkPath::addRoundRect(const SkRect& rect, float rx, float ry,
		SkPathDirection ) {
	addRect(rect);  // !!! incomplete
}

const SkPath& SkPath::makeTransform(SkMatrix const & m) { 
       for (TinyCurve& c : path) {
               m.mapPoints(c.pts, (int) c.pointCount());
       }
       return *this;
}

// !!! not sure that this is right
void SkPath::transform(const SkMatrix& matrix, SkPath* dst) {
	if (dst)
		dst->makeTransform(matrix);
	else
		makeTransform(matrix);
}

void SkPath::offset(float dx, float dy) {
	OpVector dxy(dx, dy);
	for (TinyCurve& c : path) {
		for (size_t index = 0; index < c.pointCount(); ++index)
			c.pts[index] += dxy;
	}
}

#if OP_DEBUG_SERIALIZE

std::string SkPath::fillTypeStr() const {
	std::array<std::string, 4> ws { "kWinding", "kEvenOdd", "kInverseWinding",  "kInverseEvenOdd" };
	if (SkPathFillType::kWinding <= fFillType && fFillType <= SkPathFillType::kInverseEvenOdd)
		return "setFillType(SkPathFillType::" + ws[(int) fFillType] + ");\n";
	OP_ASSERT(0);
	return "setFillType((SkPathFillType) " + STR((int) fFillType) + ");\n";
}

std::string SkPath::debugDumpCommon(bool hex, std::string callPrefix) const {
	std::string result;
	result += callPrefix + fillTypeStr();
	bool move = true;
	OpPoint first;
	auto num = [hex](float f) {
		return hex ? OpDebugDumpHex(f) : STR(f);
	};
	for (const TinyCurve& c : path) {
		if (move) {
			result += callPrefix + "moveTo(" + num(c.pts[0].x) + ", " + num(c.pts[0].y) + ");\n";
			first = c.pts[0];
		}
		move = false;
		switch (c.type) {
			case TinyType::line:
				result += callPrefix + "lineTo(" + num(c.pts[1].x) + ", " + num(c.pts[1].y) + ");\n";
				break;
			case TinyType::quad:
				result += callPrefix + "quadTo(" + num(c.pts[1].x) + ", " + num(c.pts[1].y) + ", " 
					+ num(c.pts[2].x) + ", " + num(c.pts[2].y) + ");\n";
				break;
			case TinyType::conic:
				result += callPrefix + "conicTo(" + num(c.pts[1].x) + ", " + num(c.pts[1].y) + ", " 
					+ num(c.pts[2].x) + ", " + num(c.pts[2].y) + ", " + num(c.weight) + ");\n";
				break;
			case TinyType::cubic:
				result += callPrefix + "cubicTo(" + num(c.pts[1].x) + ", " + num(c.pts[1].y) + ", " 
					+ num(c.pts[2].x) + ", " + num(c.pts[2].y) + ", "
					+ num(c.pts[3].x) + ", " + num(c.pts[3].y) + ");\n";
				break;
		}
		if (first == c.lastPt()) {
			result += callPrefix + "close();\n";
			first = c.lastPt();
			move = true;
		}
	}
	if (!result.empty() && '\n' == result.back())
		result.pop_back();
	return result;
}

void SkPath::dumpCommon(bool hex, std::string prefix) const {
	std::string s = debugDumpCommon(hex, prefix);
	OpDebugOut(s);
}

void SkPath::dump() const {
	dumpCommon(false, "path.");
}

void SkPath::dumpHex() const {
	 dumpCommon(true, "path.");
}

#endif
