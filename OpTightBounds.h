#ifndef OpTightBounds_DEFINED
#define OpTightBounds_DEFINED

#include "OpCurve.h"

enum class SectReason {
	coinPtsMatch,
	curveCurveUnsectable,
	degenerateCenter,
	divideAndConquer_oneT,
	divideAndConquer_noEdgeToSplit,
	divideAndConquer_noOppToSplit,
	endPt,
	inflection,
	lineCurve,
	missingCoincidence,
	resolveCoin_windingChange,
	resolveCoin_oWindingChange,
	sharedEdge,
	sharedEnd,
    sharedStart,
	startPt,
	xExtrema,
	yExtrema,
	// testing only
	test,
};

struct ExtremaT {
    ExtremaT(OpPtT pt_T  OP_DEBUG_PARAMS(SectReason r)) 
        : ptT(pt_T)
        OP_DEBUG_PARAMS(reason(r)) {
    }

    OpPtT ptT;
    OP_DEBUG_CODE(SectReason reason;)
};

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

    OpPoint add(OpPoint pt) {
        OP_ASSERT(pt.isFinite());
        left = std::min(left, pt.x);
        top = std::min(top, pt.y);
        right = std::max(right, pt.x);
        bottom = std::max(bottom, pt.y);
        return pt;
    }

    OpPointBounds& add(const OpPointBounds& bounds) {
        OP_ASSERT(bounds.isFinite());
        left = std::min(left, bounds.left);
        top = std::min(top, bounds.top);
        right = std::max(right, bounds.right);
        bottom = std::max(bottom, bounds.bottom);
        return *this;
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

    void pin(OpPoint* pt) {
        pt->x = OpMath::Pin(left, pt->x, right);
        pt->y = OpMath::Pin(top, pt->y, bottom);
    }

    void set(const OpCurve& c) {
        set(c.pts, c.pointCount());
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
        debugValidate();
        r.debugValidate();
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

};

struct OpTightBounds : OpPointBounds {
    OpTightBounds() {}
    OpTightBounds(OpCurve& curve) {
        set(curve);
    }

    void calcBounds(const OpQuad& quad) {
        OpPointBounds::set(quad.pts[0], quad.pts[2]);
        if (!quad.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = quad.extrema(XyChoice::inX);
            *xExtrema = { add(quad.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
        }
        if (!quad.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = quad.extrema(XyChoice::inY);
            *yExtrema = { add(quad.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
        }
    }

    void calcBounds(const OpConic& conic) {
        OpPointBounds::set(conic.pts[0], conic.pts[2]);
        if (!conic.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = conic.extrema(XyChoice::inX);
            *xExtrema = { add(conic.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
        }
        if (!conic.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = conic.extrema(XyChoice::inY);
            *yExtrema = { add(conic.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
        }
    }

    void calcBounds(OpCubic& cubic) {
        OpPointBounds::set(cubic.pts[0], cubic.pts[3]);
        if (!cubic.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = cubic.extrema(XyChoice::inX);
            int count = extremaTs.count;
            if (!count)
                cubic.pinCtrls(XyChoice::inX);
            while (count--) {
                xExtrema[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
            }
        }
        if (!cubic.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = cubic.extrema(XyChoice::inY);
            int count = extremaTs.count;
            if (!extremaTs.count)
                cubic.pinCtrls(XyChoice::inY);
              while (count--) {
                yExtrema[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
            }
        }
        OpRoots extremaTs = cubic.inflections();
        int count = extremaTs.count;
        while (count--) {
            inflections[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
        }
    }
    
    std::vector<ExtremaT> findExtrema(OpPoint start, OpPoint end) {
        std::vector<ExtremaT> selfPtTs;
        for (size_t index = 0; index < ARRAY_COUNT(xExtrema); ++index) {
            if (OpMath::IsNaN(xExtrema[index].t))
                break;
            if (xExtrema[index].pt == start || xExtrema[index].pt == end)
                continue;
            selfPtTs.emplace_back(xExtrema[index]  OP_DEBUG_PARAMS(SectReason::xExtrema));
        }
        for (size_t index = 0; index < ARRAY_COUNT(yExtrema); ++index) {
            if (OpMath::IsNaN(yExtrema[index].t))
                break;
            if (yExtrema[index].pt == start || yExtrema[index].pt == end)
                continue;
            selfPtTs.emplace_back(yExtrema[index]  OP_DEBUG_PARAMS(SectReason::yExtrema));
        }
        for (size_t index = 0; index < ARRAY_COUNT(inflections); ++index) {
            if (OpMath::IsNaN(inflections[index].t))
                break;
            if (inflections[index].pt == start || inflections[index].pt == end)
                continue;
            selfPtTs.emplace_back(inflections[index]  OP_DEBUG_PARAMS(SectReason::inflection));
        }
        std::sort(selfPtTs.begin(), selfPtTs.end(), [](const ExtremaT& s1, const ExtremaT& s2) {
            return s1.ptT.t < s2.ptT.t; });

        return selfPtTs;
    }

    void set(OpCurve& c) {
        switch (c.type) {
        case OpType::line: OpPointBounds::set(c.pts, 2); break;
        case OpType::quad: calcBounds(c.asQuad()); break;
        case OpType::conic: calcBounds(c.asConic()); break;
        case OpType::cubic: calcBounds(c.asCubic()); break;
        default: OP_ASSERT(0);
        }
    }

    void set(const OpRect& rect) {
        left = rect.left;
        top = rect.top;
        right = rect.right;
        bottom = rect.bottom;
    }

#if OP_DEBUG_DUMP
    std::string debugDump() const {
        std::string s = OpRect::debugDump();
        if (!OpMath::IsNaN(xExtrema[0].t) || !OpMath::IsNaN(xExtrema[1].t))
            s += "\n    xExtrema: " + xExtrema[0].debugDump() + ", " + xExtrema[1].debugDump();
        if (!OpMath::IsNaN(yExtrema[0].t) || !OpMath::IsNaN(yExtrema[1].t))
            s += "\n    yExtrema: " + yExtrema[0].debugDump() + ", " + yExtrema[1].debugDump();
        if (!OpMath::IsNaN(inflections[0].t) || !OpMath::IsNaN(inflections[1].t))
            s += "\n    inflections: " + inflections[0].debugDump() + ", " + inflections[1].debugDump();
        return s;
    }
#endif

    OpPtT xExtrema[2];
    OpPtT yExtrema[2];
    OpPtT inflections[2];
};

#endif