// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpTightBounds_DEFINED
#define OpTightBounds_DEFINED

#include "OpCurve.h"

enum class SectReason {
    unset,
	coinPtsMatch,
	curveCurveUnsectable,
	degenerateCenter,
	divideAndConquer_oneT,
	divideAndConquer_noEdgeToSplit,
	divideAndConquer_noOppToSplit,
	edgeCCExact,
	edgeCCHull,
	edgeCCHullPair,
	edgeCCNearly,
	edgeCurveCurve,
	endPt,
	inflection,
	lineCurve,
	missingCoincidence,
	oppCCExact,
	oppCCHull,
	oppCCHullPair,
	oppCCNearly,
	oppCurveCurve,
	resolveCoin_windingChange,
	resolveCoin_oWindingChange,
	sharedEdge,
	sharedEnd,
    sharedStart,
    soClose,
	startPt,
    unsectableOppEnd,
    unsectableOppStart,
    unsectableEnd,
    unsectableStart,
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

    OpPointBounds(OpPoint pt1, OpPoint pt2) {
        set(pt1);
        add(pt2);
    }

    OpPoint add(OpPoint pt) {
        if (!pt.isFinite())
            return OpPoint(SetToNaN::dummy);
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

    bool nearlyContains(OpPoint pt) const {
        OP_ASSERT(pt.isFinite());
        return OpMath::Betweenish(left, pt.x, right) && OpMath::Betweenish(top, pt.y, bottom);
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

struct OpTightBounds : OpPointBounds {
    OpTightBounds() {
#if OP_DEBUG
        debugXExtremaFailed = false;
        debugYExtremaFailed = false;
#endif
    }
    OpTightBounds(OpCurve& curve) {
        set(curve);
    }

    bool calcBounds(const OpQuad& quad) {
        OpPointBounds::set(quad.pts[0], quad.pts[2]);
        if (!quad.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = quad.extrema(XyChoice::inX);
            if (extremaTs.count) {
                OP_ASSERT(extremaTs.count == 1);
                *xExtrema = { add(quad.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
                if (!xExtrema->pt.isFinite())
                    return false;
            } else  // fuzz may trigger, but I suspect non-fuzz could fail as well; defer total failure for now
                OP_DEBUG_CODE(debugXExtremaFailed = true);  // triggered by fuzz767834
        }
        if (!quad.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = quad.extrema(XyChoice::inY);
            if (extremaTs.count) {
                OP_ASSERT(extremaTs.count == 1);
                *yExtrema = { add(quad.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
                if (!yExtrema->pt.isFinite())
                    return false;
            } else  // see above
                OP_DEBUG_CODE(debugYExtremaFailed = true);  // triggered by fuzz767834
        }
        return true;
    }

    bool calcBounds(const OpConic& conic) {
        OpPointBounds::set(conic.pts[0], conic.pts[2]);
        if (!conic.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = conic.extrema(XyChoice::inX);
            if (extremaTs.count) {
                OP_ASSERT(extremaTs.count == 1);
                *xExtrema = { add(conic.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
                if (!xExtrema->pt.isFinite())
                    return false;
            } else  // see above
                OP_DEBUG_CODE(debugXExtremaFailed = true);  // triggered by fuzzhang_2
        }
        if (!conic.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = conic.extrema(XyChoice::inY);
            if (extremaTs.count) {
                OP_ASSERT(extremaTs.count == 1);
                *yExtrema = { add(conic.ptAtT(extremaTs.roots[0])), extremaTs.roots[0] };
                if (!yExtrema->pt.isFinite())
                    return false;
            } else  // see above
                OP_DEBUG_CODE(debugYExtremaFailed = true);  // triggered by fuzz754434_2
        }
        return true;
    }

    bool calcBounds(OpCubic& cubic) {
        OpPointBounds::set(cubic.pts[0], cubic.pts[3]);
        if (!cubic.monotonic(XyChoice::inX)) {
            OpRoots extremaTs = cubic.extrema(XyChoice::inX);
            if (int count = extremaTs.count) {
                OP_ASSERT(extremaTs.count < 3);
    //          if (!count)  // if there's no extrema, pinning the control points on the original cubic is wrong (cubicOp66u)
    //              cubic.pinCtrls(XyChoice::inX);  // if needed, document when and why
                while (count--) {
                    xExtrema[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
                    if (!xExtrema[count].pt.isFinite())
                        return false;
                }
            } else  // see above
                OP_DEBUG_CODE(debugXExtremaFailed = true);  // triggered by pentrek4
        }
        if (!cubic.monotonic(XyChoice::inY)) {
            OpRoots extremaTs = cubic.extrema(XyChoice::inY);
            if (int count = extremaTs.count) {
                OP_ASSERT(extremaTs.count < 3);
    //          if (!count)
    //              cubic.pinCtrls(XyChoice::inY);
                while (count--) {
                    yExtrema[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
                    if (!yExtrema[count].pt.isFinite())
                        return false;
                }
            } else  // see above
                OP_DEBUG_CODE(debugYExtremaFailed = true);  // triggered by pentrek7
        }
        OpRoots extremaTs = cubic.inflections();
        int count = extremaTs.count;
        while (count--) {
            inflections[count] = { add(cubic.ptAtT(extremaTs.roots[count])), extremaTs.roots[count] };
        }
        return true;
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
#if OP_DEBUG
        debugXExtremaFailed = false;
        debugYExtremaFailed = false;
#endif
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
    ~OpTightBounds() override {}
    OpTightBounds(const OpTightBounds& p) = default;
    OpTightBounds& operator=(const OpTightBounds& p) = default;
    DUMP_DECLARATIONS_OVERRIDE
#endif

    OpPtT xExtrema[2];
    OpPtT yExtrema[2];
    OpPtT inflections[2];
#if OP_DEBUG
    // set when curve is not monotonic but extrema calculation returned zero roots
    bool debugXExtremaFailed;
    bool debugYExtremaFailed;
#endif
};

#endif
