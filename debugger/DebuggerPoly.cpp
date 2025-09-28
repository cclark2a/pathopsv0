// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpDebugPicture.h"
#include "OpContour.h"
#include "OpSegment.h"

void DebuggerAddPoly::add(const PathOpsV0Lib::Curve& c) {
    if (c.data->start == c.data->end)
        return;
    // if adding fill, wrap curve to focus bounds
    OpRect bounds(c.data->start, c.data->end);
    auto addVertical = [c, bounds, this](float x) {
        if (!addingFill)
            return;
        float top = std::max(bounds.top, picture->focus.top);
        float bottom = std::min(bounds.bottom, picture->focus.bottom);
        if (top >= bottom)
            return;
        if (c.data->start.y > c.data->end.y)
            std::swap(top, bottom);
        picture->add({ x, top }, { x, bottom }, this);
    };
    auto addHorizontal = [c, bounds, this](float y) {
        if (!addingFill)
            return;
        float left = std::max(bounds.left, picture->focus.left);
        float right = std::min(bounds.right, picture->focus.right);
        if (left >= right)
            return;
        if (c.data->start.x > c.data->end.x)
            std::swap(left, right);
        picture->add({ left, y }, { right, y }, this);
    };
    if (bounds.right <= picture->focus.left)
        return addVertical(picture->focus.left);
    if (bounds.left >= picture->focus.right)
        return addVertical(picture->focus.right);
    if (bounds.bottom <= picture->focus.top)
        return addHorizontal(picture->focus.top);
    if (bounds.top >= picture->focus.bottom)
        return addHorizontal(picture->focus.bottom);
    OpCurve curve(c, Rotated::no);
    if (picture->focus.contains(bounds)) {
        picture->add(curve, this);
        return;
    }
    std::vector<DebugSect> sects;
    // bounds overlaps, but curve may not intersect; find interior ends, intersection with bounds
    auto addPin = [this, &sects](OpPtT end) {
        OpPoint sect = { OpMath::PinSorted(picture->focus.left, end.pt.x, picture->focus.right),
                OpMath::PinSorted(picture->focus.top, end.pt.y, picture->focus.bottom) };
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
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.left), picture->focus.left,
            Axis::vertical);
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.top), picture->focus.top,
            Axis::horizontal);
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.right), picture->focus.right,
            Axis::vertical);
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.bottom), picture->focus.bottom,
            Axis::horizontal);
    addPin(OpPtT(c.data->end, 1));
    // for each span : if middle is inside focus, keep ends of span
    std::sort(sects.begin(), sects.end(), [](const auto& s1, const auto& s2) {
			return s1.sect.t < s2.sect.t; } );
    DebugSect* last = &sects.front();
    for (DebugSect& sect : sects) {
        if (last->sect.t < sect.sect.t) {
            if (last->pin || sect.pin) {
                if (addingFill && last->sect.pt != sect.sect.pt)
                    picture->add(last->sect.pt, sect.sect.pt, this);
            } else {
                OpCurve piece = curve.subDivide(last->sect.t, sect.sect.t);
                piece.setFirstPt(last->sect.pt);
                piece.setLastPt(sect.sect.pt);
                picture->add(piece, this);
                DebuggerPoly& added = picture->polys.back();
                added.tStart = last->sect.t;
                added.tEnd = sect.sect.t;
            }
        }
        last = &sect;
    }
}

void DebuggerAddPoly::add(const OpEdge* e) {
    edge = e;
    segment = nullptr;
    contour = nullptr;
    addingFill = false;
    add(e->curve.c);
}

void DebuggerAddPoly::add(const OpSegment* s) {
    edge = nullptr;
    segment = s;
    contour = nullptr;
    addingFill = false;
    add(s->c.c);
}

// !!! for segments making up area; color comes from contour
void DebuggerAddPoly::add(const OpContour* c) {
    edge = nullptr;
    segment = nullptr;
    contour = c;
    addingFill = true;
    OpPoint last(SetToNaN::dummy);
    for (const PathOpsV0Lib::Curve& curve : c->debugCurves) {
        continueCurve = last == curve.data->start;
        add(curve);
        last = curve.data->end;
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
    if (edge)
        s += "edge:" + STR(edge->id) + " ";
    if (segment)
        s += "segment:" + STR(segment->id) + " ";
    if (contour)
        s += "contour:" + STR(contour->id) + " ";
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
