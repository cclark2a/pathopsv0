#ifndef OpTightBounds_DEFINED
#define OpTightBounds_DEFINED

#include "OpCurve.h"

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
        assert(pt.isFinite());
        left = std::min(left, pt.x);
        top = std::min(top, pt.y);
        right = std::max(right, pt.x);
        bottom = std::max(bottom, pt.y);
        return pt;
    }

    void add(const OpPointBounds& bounds) {
        assert(bounds.isFinite());
        left = std::min(left, bounds.left);
        top = std::min(top, bounds.top);
        right = std::max(right, bounds.right);
        bottom = std::max(bottom, bounds.bottom);
    }

    bool contains(OpPoint pt) const {
        assert(pt.isFinite());
        return OpMath::Between(left, pt.x, right) && OpMath::Between(top, pt.y, bottom);
    }

    OpPointBounds intersect(const OpPointBounds& bounds) const {
        assert(bounds.isFinite());
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
//        assert(pt.isFinite());    // in debug code, points may be NaN
        return OpMath::Between(left, pt.x, right) && OpMath::Between(top, pt.y, bottom);
    }

    bool debugContains(const OpPointBounds& bounds) {
        assert(bounds.isFinite());
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

    void pin(OpPoint* pt) {
        pt->x = OpMath::Pin(left, pt->x, right);
        pt->y = OpMath::Pin(top, pt->y, bottom);
    }

    void set(OpCurve& c) {
        switch (c.type) {
        case pointType: OpPointBounds::set(c.pts, 1); break;
        case lineType: OpPointBounds::set(c.pts, 2); break;
        case quadType: calcBounds(c.asQuad()); break;
        case conicType: calcBounds(c.asConic()); break;
        case cubicType: calcBounds(c.asCubic()); break;
        default: assert(0);
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