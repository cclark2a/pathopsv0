// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTightBounds_DEFINED
#define OpTightBounds_DEFINED

#include "OpMath.h"

struct OpPointBounds : OpRect {
	OpPointBounds() {
		left = +OpInfinity;
		top = +OpInfinity;
		right = -OpInfinity;
		bottom = -OpInfinity;
	}

	OpPointBounds(float l, float t, float r, float b)
		: OpRect(l, t, r, b) {
	}

	OpPointBounds(OpPoint pt1, OpPoint pt2) {
		set(pt1);
		add(pt2);
	}

	bool contains(OpPoint pt) const {
		OP_ASSERT(pt.isFinite());
		return OpMath::Between(left, pt.x, right) && OpMath::Between(top, pt.y, bottom);
	}

	OpPointBounds intersect(const OpPointBounds& bounds) const {
		OP_ASSERT(bounds.isFinite());
		return {
			std::max(left, bounds.left),
			std::max(top, bounds.top),
			std::min(right, bounds.right),
			std::min(bottom, bounds.bottom)
		};
	}

	bool isEmpty() const {
		return left >= right && top >= bottom;
	}

	bool isSet() const {
		return OpMath::IsFinite(left);
	}

	// !!! convert this to use threshold instead
	bool nearlyContains(OpPoint pt, OpVector threshold) const {
		OP_ASSERT(pt.isFinite());
		return OpMath::InSorted(left, pt.x, right, threshold.dx)
				&& OpMath::InSorted(top, pt.y, bottom, threshold.dy);
	}

	// used to check if pair describe gap between edges
	// tricky: if gaps are axis-aligned lines, look only at one coordinate
	bool overlaps(const OpPointBounds& r) const {
#if OP_DEBUG_VALIDATE
		debugValidate();
		r.debugValidate();
#endif
		bool overlapsInX = r.left < right && left < r.right;
		bool overlapsInY = r.top < bottom && top < r.bottom;
		if (top == bottom)
			return overlapsInX && r.top <= top && top <= r.bottom;
		if (left == right)
			return overlapsInY && r.left <= left && left <= r.right;
		return overlapsInX && overlapsInY;
	}

	void pin(OpPoint* pt) {
		pt->x = OpMath::PinSorted(left, pt->x, right);
		pt->y = OpMath::PinSorted(top, pt->y, bottom);
	}

	OpPoint set(OpPoint pt) {
		left = right = pt.x;
		top = bottom = pt.y;
		return pt;
	}

	void set(OpPoint pt1, OpPoint pt2) {
		set(pt1);
		add(pt2);
	}

	void set(const OpPoint* pts, int count) {
		for (int index = 0; index < count; ++index) {
			add(pts[index]);
		}
	}

	bool touches(const OpPointBounds& r) const {
#if OP_DEBUG_VALIDATE
		debugValidate();
		r.debugValidate();
#endif
		return r.left == right || left == r.right || r.top == bottom || top == r.bottom;
	}

#if OP_DEBUG
	bool debugContains(OpPoint pt) {
//        OP_ASSERT(pt.isFinite());    // in debug code, points may be NaN
		return OpMath::Between(left, pt.x, right) && OpMath::Between(top, pt.y, bottom);
	}

	bool debugContains(const OpPointBounds& bounds) {
		OP_ASSERT(bounds.isFinite());
		return OpMath::Between(left, bounds.left, right) && OpMath::Between(left, bounds.right, right)
				&& OpMath::Between(top, bounds.top, bottom) && OpMath::Between(top, bounds.bottom, bottom);
	}
#endif

#if OP_DEBUG_DUMP
	~OpPointBounds() override {}
	OpPointBounds(const OpPointBounds& p) = default;
	OpPointBounds& operator=(const OpPointBounds& p) = default;
	DUMP_DECLARATIONS_OVERRIDE
#endif

};

#endif
