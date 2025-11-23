// (c) 2025, Cary Clark cclark2@gmail.com

#include "DebuggerState.h"
#include "OpContext.h"
#include "OpSegment.h"
#include "DebugOpsTypes.h"

struct DebugSect {  // curve intersected with focus rectangle, and intersection pinned to rect
    OpPtT sect;
    bool pin;
};

void DebuggerAddPoly::add(const PathOpsV0Lib::Curve& c) {
    if (c.data->start == c.data->end)
        return;
    // if adding fill, wrap curve to focus bounds
    OpCurve curve(c, monotonic ? Rotated::no : Rotated::yes);
    OpPointBounds bounds = curve.ptBounds();
    auto addVertical = [c, bounds, this](float x) {
        if (!addingFill)
            return;
        float top = std::max(bounds.top, window->focus.top);
        float bottom = std::min(bounds.bottom, window->focus.bottom);
        if (top >= bottom)
            return;
        if (c.data->start.y > c.data->end.y)
            std::swap(top, bottom);
        window->add({ x, top }, { x, bottom }, this);
    };
    auto addHorizontal = [c, bounds, this](float y) {
        if (!addingFill)
            return;
        float left = std::max(bounds.left, window->focus.left);
        float right = std::min(bounds.right, window->focus.right);
        if (left >= right)
            return;
        if (c.data->start.x > c.data->end.x)
            std::swap(left, right);
        window->add({ left, y }, { right, y }, this);
    };
    if (bounds.right <= window->focus.left)
        return addVertical(window->focus.left);
    if (bounds.left >= window->focus.right)
        return addVertical(window->focus.right);
    if (bounds.bottom <= window->focus.top)
        return addHorizontal(window->focus.top);
    if (bounds.top >= window->focus.bottom)
        return addHorizontal(window->focus.bottom);
    if (window->focus.contains(bounds)) {
        window->add(curve, this);
        return;
    }
    std::vector<DebugSect> sects;
    // bounds overlaps, but curve may not intersect; find interior ends, intersection with bounds
    auto addPin = [this, &sects](OpPtT end) {
        OpPoint sect = { OpMath::PinSorted(window->focus.left, end.pt.x, window->focus.right),
                OpMath::PinSorted(window->focus.top, end.pt.y, window->focus.bottom) };
        sects.push_back({{ sect, end.t }, sect != end.pt });
    };
    auto addSects = [&curve, addPin](OpRoots roots, float xy, Axis axis) {
        for (float root : roots.roots) {
            OpPtT ptAtT = curve.ptTAtT(root);
            ptAtT.pt.choice(axis) = xy;
            addPin(ptAtT);
        }
    };
    addPin(OpPtT(c.data->start, 0));
    bool debugThis = IDType::segment == opType.type && 5 == opType.segment->id 
            && opType.segment->ptBounds.top < window->focus.top;
    auto crossRoots = [&curve](Axis axis, float xy) {
        if (!OpMath::Between(curve.firstPt().choice(axis), xy, curve.lastPt().choice(axis)))
            return OpRoots();
        return curve.axisRayHit(axis, xy);
    };
    addSects(crossRoots(Axis::vertical, window->focus.left), window->focus.left,
            Axis::vertical);
    addSects(crossRoots(Axis::horizontal, window->focus.top), window->focus.top,
            Axis::horizontal);
    addSects(crossRoots(Axis::vertical, window->focus.right), window->focus.right,
            Axis::vertical);
    if (debugThis)
        OpNop();
    addSects(crossRoots(Axis::horizontal, window->focus.bottom), window->focus.bottom,
            Axis::horizontal);
    addPin(OpPtT(c.data->end, 1));
    // for each span : if middle is inside focus, keep ends of span
    std::sort(sects.begin(), sects.end(), [](const auto& s1, const auto& s2) {
			return s1.sect.t < s2.sect.t; } );
    DebugSect* last = &sects.front();
    for (DebugSect& sect : sects) {
        if (last->sect.t < sect.sect.t) {
            OpCurve piece = curve.debugSubDivide(last->sect.t, sect.sect.t);
            if (debugThis)
                OpNop();
            OpPointBounds pieceBounds = piece.ptBounds();
            bool overlaps = pieceBounds.intersectsThreshold(window->focus, -window->threshold);
            if (!overlaps) {
                if (addingFill && last->sect.pt != sect.sect.pt)
                    window->add(last->sect.pt, sect.sect.pt, this);
            } else {
                piece.setFirstPt(last->sect.pt);
                piece.setLastPt(sect.sect.pt);
                window->add(piece, this);
                DebuggerPoly& added = window->polys.back();
                added.tStart = last->sect.t;
                added.tEnd = sect.sect.t;
            }
        }
        last = &sect;
    }
    if (debugThis)
        OpNop();
}

void DebuggerAddPoly::add(const OpEdge* e) {
    opType = OpType(e);
    addingFill = false;
    monotonic = true;
    add(e->curve.c);
}

void DebuggerAddPoly::add(const OpSegment* s) {
    opType = OpType(s);
    addingFill = false;
    monotonic = true;
    add(s->c.c);
}

// !!! for segments making up area; color comes from contour
void DebuggerAddPoly::add(const OpContour* c) {
    opType = OpType(c);
    addingFill = true;
    monotonic = false;
    OpPoint last(SetToNaN::dummy);
    for (curveIndex = 0; curveIndex < (int) c->debugCurveData.size(); ++curveIndex) {
        std::vector<float> extrema;
        OpCurve opCurve = c->debugCurve(curveIndex, &extrema);
        OpRoots tValues;
        for (float ex : extrema) {
            tValues.add(ex);
        }
        tValues.add(0);
        tValues.add(1);
        tValues.sort();
        for (int index = 0; index < tValues.count() - 1; ++index) {
            OpCurve piece = opCurve.subDivide(tValues.roots[index], tValues.roots[index + 1]);
            continueCurve = last == piece.firstPt();
            add(piece.c);
            last = piece.lastPt();
        }   
    }
    continueCurve = false;
}

#if 0
void DebuggerAddPoly::add(const LinePts& pts) {    
    std::vector<OpPoint> points { pts.pts[0], pts.pts[1] };
    return picture->add(points);
}

void DebuggerAddPoly::add(const OpRect& r) {
    std::vector<OpPoint> points { 
        { r.left, r.top }, { r.left, r.bottom }, { r.right, r.bottom }, { r.right, r.top } };
    return picture->add(points);
}
#endif

#if OP_DEBUG_DUMP

extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

void DebuggerPoly::dump() const {
    std::string s;
    s += "local:" + STR(local.size()) + " ";
    if (IDType::edge == opType.type)
        s += "edge:";
    if (IDType::segment == opType.type)
        s += "segment:";
    if (opType.id)
    s += STR(opType.id) + " ";
    if (1 != thickness)
        s += " thickness:" + STR(thickness) + " ";
    if (debugBlack != color)
        s += "color:" + debugDumpColor(defaultLevel, color) + " ";
    if (0 != tStart)
        s += "tStart:" + STR(tStart) + " ";
    if (1 != tEnd)
        s += "tEnd:" + STR(tEnd) + " ";
    if (isPrimary)
        s += "isPrimary ";
    s.pop_back();
    s += "\n";
    for (OpPoint pt : local) {
         s += pt.debugDump(defaultLevel, defaultBase) + "\n";
    }
    OpDebugOut(s);
}

#endif
