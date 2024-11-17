// (c) 2023, Cary Clark cclark2@gmail.com
#include <cstdarg>
#include <cmath>
#include "TinySkia.h"
#include "OpTightBounds.h"


#if _WIN32
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

bool SkRect::contains(SkRect const & r) { 
	return fLeft <= r.fLeft && fRight >= r.fRight && fTop <= r.fTop && fBottom >= r.fBottom; 
}

bool SkRect::intersect(SkRect const & r) { 
	return fLeft < r.fRight && fRight > r.fLeft && fTop < r.fBottom && fBottom > r.fTop; 
}

void SkRect::join(SkRect const & r) {
	fLeft = std::min(fLeft, r.fLeft);
	fTop = std::min(fTop, r.fTop);
	fRight = std::max(fRight, r.fRight);
	fBottom = std::max(fBottom, r.fBottom);
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

void SkMatrix::reset() { 
	m[0][0] = 1;
	m[0][1] = 0;
	m[0][2] = 0;
	m[1][0] = 0;
	m[1][1] = 1;
	m[1][2] = 0;
}

void SkMatrix::setScale(float sx, float sy) { 
	m[0][0] = sx;
	m[0][1] = 0;
	m[0][2] = 0;
	m[1][0] = 0;
	m[1][1] = sy;
	m[1][2] = 0;
}

void SkMatrix::setRotate(float deg) {
	float rad = deg * (OpPI / 180);
	float cos = COS_F(rad);
	float sin = SIN_F(rad);
	m[0][0] = cos;
	m[0][1] = -sin;
	m[0][2] = 0;
	m[1][0] = sin;
	m[1][1] = cos;
	m[1][2] = 0;
}

void SkMatrix::setRotate(float deg, float px, float py) {
	float rad = deg * (OpPI / 180);
	float cos = COS_F(rad);
	float sin = SIN_F(rad);
	m[0][0] = cos;
	m[0][1] = -sin;
	m[0][2] = sin * py + (1 - cos) * px;
	m[1][0] = sin;
	m[1][1] = cos;
	m[1][2] = -sin * px + (1 - cos) * py;
}

void SkMatrix::preScale(float sx, float sy) { 
	SkMatrix tmp;
	tmp.reset();
	tmp.m[0][0] = sx;
	tmp.m[1][1] = sy;
	setConcat(*this, tmp);
}

void SkMatrix::preTranslate(float dx, float dy) { 
	SkMatrix tmp;
	tmp.reset();
	tmp.m[0][2] = dx;
	tmp.m[1][2] = dy;
	setConcat(*this, tmp);
}

void SkMatrix::postTranslate(float dx, float dy) { 
	SkMatrix tmp;
	tmp.reset();
	tmp.m[0][2] = dx;
	tmp.m[1][2] = dy;
	setConcat(tmp, *this);
}

void SkMatrix::mapPoints(SkPoint* pts, int count) const {
	mapPoints((OpPoint*) pts, count);
}

void SkMatrix::mapPoints(OpPoint* pts, int count) const {
	for (int index = 0; index < count; ++index) {
		OpPoint pt = pts[index];
		pts[index].x = pt.x * m[0][0] + pt.y * m[0][1] + m[0][2];
		pts[index].y = pt.x * m[1][0] + pt.y * m[1][1] + m[1][2];
	}
}

void SkMatrix::setConcat(const SkMatrix& a, const SkMatrix& b) {
	auto muladdmul = [](float a, float b, float c, float d) {
		return a * b + c * d;
	};
	SkMatrix tmp;
    tmp.m[0][0] = muladdmul(a.m[0][0],
                            b.m[0][0],
                            a.m[0][1],
                            b.m[1][0]);

    tmp.m[0][1] = muladdmul(a.m[0][0],
                            b.m[0][1],
                            a.m[0][1],
                            b.m[1][1]);

    tmp.m[0][2] = muladdmul(a.m[0][0],
                            b.m[0][2],
                            a.m[0][1],
                            b.m[1][2]) + a.m[0][2];

    tmp.m[1][0] = muladdmul(a.m[1][0],
                            b.m[0][0],
                            a.m[1][1],
                            b.m[1][0]);

    tmp.m[1][1] = muladdmul(a.m[1][0],
                            b.m[0][1],
                            a.m[1][1],
                            b.m[1][1]);

    tmp.m[1][2] = muladdmul(a.m[1][0],
                            b.m[0][2],
                            a.m[1][1],
                            b.m[1][2]) + a.m[1][2];
	*this = tmp;
}

void SkBitmap::allocPixels(struct SkImageInfo const & i) {
	info = i;
	delete pixels;
	pixels = new uint32_t[i.width * i.height];
}

uint32_t SkBitmap::getColor(int x, int y) {
	OP_ASSERT(x < info.width);
	OP_ASSERT(y < info.height);
	OP_ASSERT(pixels);
	return pixels[info.width * y + x]; 
}

void SkCanvas::save() { 
	m.emplace_back();
	m.back() = *(&m.back() - 1);
}

void SkCanvas::restore() {
	OP_ASSERT(m.size() > 1);
	m.pop_back();
}

void SkCanvas::rotate(float degrees, float px, float py) {
    SkMatrix tmp;
    tmp.setRotate(degrees, px, py);
	SkMatrix& top = m.back();
    top.setConcat(top, tmp);
}

void SkCanvas::translate(float dx, float dy) {
	SkMatrix& top = m.back();
    top.preTranslate(dx, dy);
}

void SkCanvas::drawLine(float x1, float y1, float x2, float y2, const SkPaint& paint) {
	OpPoint start(x1, y1);
	OpPoint end(x2, y2);
	// move pixel center axis to pixel center axis
}

void SkCanvas::drawPath(const SkPath& path,const SkPaint& paint) {

}

void SkCanvas::drawString(SkString s, float x, float y, const SkFont& f, const SkPaint& paint) {

}

SkFont::SkFont(void*, float s, float, float) { 
	fSize = s; 
}

float SkFont::measureText(void const *, uint64_t, SkTextEncoding, SkRect *) const { 
	return 0; 
}

void SkPath::reset() { 
	path.clear();
}

const SkRect& SkPath::getBounds() const {
	OpPointBounds b;
	for (const TinyCurve& c : path)
		for (size_t index = 0; index < c.pointCount(); ++index)
			if (!c.pts[index].debugIsUninitialized())
				b.add(c.pts[index]);
	bounds = { b.left, b.top, b.right, b.bottom };
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
		pts[0] = SkPoint::Make(c.pts[0].x, c.pts[0].y);
		return SkPath::kMove_Verb;
	}
	if (first == c.lastPt())
		nextClose = true;
	for (size_t i = 0; i < c.pointCount(); ++i)
		pts[i] = SkPoint::Make(c.pts[i].x, c.pts[i].y);
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

void SkPath::rLineTo(float rx, float ry) {
	TinyCurve curve { { last, last + OpPoint(rx, ry) }, 1, TinyType::line };
	path.push_back(std::move(curve));
	last = curve.pts[1];
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
	#if !OP_DEBUG_FAST_TEST
		OP_DEBUG_CODE(extern bool debugUseAlt);
		OP_DEBUG_CODE(debugUseAlt = true);
	#endif
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

// !!! this isn't right because it doesn't keep track of contours
int SkPath::countPoints() const {
	int count = 1;
	for (const TinyCurve& curve : path)
		count += curve.pointCount() - 1;
	return count;
}

// !!! this isn't right because it doesn't keep track of contours
SkPoint SkPath::getPoint(int index) {
	for (TinyCurve& curve : path) {
		int base = index;
		index -= curve.pointCount() - 1;
		if (index > 0)
			continue;
		return { curve.pts[base].x, curve.pts[base].y };
	}
	return { 0, 0 };
}

// !!! this isn't right because it doesn't keep track of contours
void SkPath::setPt(int index, float x, float y) {
	for (TinyCurve& curve : path) {
		int base = index;
		index -= curve.pointCount() - 1;
		if (index > 0)
			continue;
		curve.pts[base] = { x, y };
		return;
	}
}


const SkPath& SkPath::makeTransform(SkMatrix const & m) { 
	for (TinyCurve& c : path) {
		m.mapPoints(c.pts, c.pointCount());
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

void SkPath::dumpCommon(bool hex) const {
	bool move = true;
	OpPoint first;
	for (const TinyCurve& c : path) {
		if (move) {
			OpDebugOut("moveTo(" + STR(c.pts[0].x) + ", " + STR(c.pts[0].y) + ");\n");
			first = c.pts[0];
		}
		move = false;
		switch (c.type) {
			case TinyType::line:
				OpDebugOut("lineTo(" + STR(c.pts[1].x) + ", " + STR(c.pts[1].y) + ");\n");
				break;
			case TinyType::quad:
				OpDebugOut("quadTo(" + STR(c.pts[1].x) + ", " + STR(c.pts[1].y) + ", " 
					+ STR(c.pts[2].x) + ", " + STR(c.pts[2].y) + ");\n");
				break;
			case TinyType::conic:
				OpDebugOut("conicTo(" + STR(c.pts[1].x) + ", " + STR(c.pts[1].y) + ", " 
					+ STR(c.pts[2].x) + ", " + STR(c.pts[2].y) + ", " + STR(c.weight) + ");\n");
				break;
			case TinyType::cubic:
				OpDebugOut("cubicTo(" + STR(c.pts[1].x) + ", " + STR(c.pts[1].y) + ", " 
					+ STR(c.pts[2].x) + ", " + STR(c.pts[2].y) + ", "
					+ STR(c.pts[3].x) + ", " + STR(c.pts[3].y) + ");\n");
				break;
		}
		if (first == c.lastPt()) {
			OpDebugOut("close();\n");
			first = c.lastPt();
		}
	}
}

void SkPath::dump() const {
	dumpCommon(false);
}

void SkPath::dumpHex() const {
	 dumpCommon(true);
}

void SkString::appendf(const char format[], ...) {
    va_list args;
    va_start(args, format);
    int count = std::vsnprintf(nullptr, 0, format, args);
	++count; // !!! don't know why I need to do this / reserving space for trailing '\0'?
	string.resize(string.size() + count, 'x');
	std::vsnprintf(&string.front() + string.size(), count, format, args);
    va_end(args);
}

void SkString::printf(const char format[], ...) {
    va_list args;
    va_start(args, format);
    int count = std::vsnprintf(nullptr, 0, format, args);
	++count; // !!! don't know why I need to do this / reserving space for trailing '\0'?
	string.resize(count, 'x');
	std::vsnprintf(&string.front(), count, format, args);
    va_end(args);
}
