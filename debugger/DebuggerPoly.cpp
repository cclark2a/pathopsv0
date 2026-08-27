// (c) 2025, Cary Clark cclark2@gmail.com

#include "DebuggerState.h"
#include "OpContext.h"
#include "OpDebugRaster.h"
#include "OpSegment.h"
#include "DebugOpsTypes.h"

OpRect OpCurve::debugBounds(float tStart, float tEnd) const {
    if (Rotated::no == rotated)
        return OpRect(ptAtT(tStart), ptAtT(tEnd));
    if (0 == tStart && 1 == tEnd)
        return fullBounds();
    OpCurve sub = debugSubDivide(tStart, tEnd);
    return sub.fullBounds();
}

void DebuggerAddPoly::add(const OpCurve& curve, float tStart, float tEnd) {
#if DEBUG_CLIP
    auto setupClip = [this, curve, tStart, tEnd]() {
        return window->createDebugClip(opType, curve, tStart, tEnd);
    };
    auto setClip = [setupClip](const OpPtT& lastPtT, const OpPtT& ptT) {
        DebugClip* clipPtr = setupClip();
        clipPtr->clipStart = lastPtT;  // may be full span or clipped span
        clipPtr->clipEnd = ptT;
        return clipPtr;
    };
    auto setBase = [curve, tStart, tEnd, setClip]() {
        return setClip(curve.ptTAtT(tStart), curve.ptTAtT(tEnd));
    };
    auto setSide = [setClip](const OpPtT& lastPtT, const OpPtT& ptT, int lastSide, int side) {
        DebugClip* clipPtr = setClip(lastPtT, ptT);
        clipPtr->lastSide = lastSide;
        clipPtr->side = side;
    };
#endif
    OpRect bounds = curve.debugBounds(tStart, tEnd);  // curve.fullBounds();
    if (bounds.isEmpty())
        return;
    // if adding fill, wrap curve to focus bounds
    const OpRect& focus = window->focus;
    auto addVertical = [curve, bounds, tStart, tEnd, focus, this  CLIP_PARAM(setBase)](float x) {
        if (!addingFill)
            return;
        float top = std::max(bounds.top, focus.top);
        float bottom = std::min(bounds.bottom, focus.bottom);
        if (top >= bottom) {
#if DEBUG_CLIP
            DebugClip* clipPtr = setBase();
            clipPtr->clipCorner = bounds.right <= focus.left
                   ? (bounds.bottom <= focus.top ? ClipCorner::topLeft : ClipCorner::bottomLeft)
                   : (bounds.bottom <= focus.top ? ClipCorner::topRight : ClipCorner::bottomRight);
#endif
            return;
        }
        if (curve.c.data->start.y > curve.c.data->end.y)
            std::swap(top, bottom);
        window->add(this, OpPtT({ x, top }, tStart), OpPtT({ x, bottom }, tEnd)  
                CLIP_PARAM(curve, tStart, tEnd));
#if DEBUG_CLIP
        DebugClip* clipPtr = setBase();
        clipPtr->clipEdge = focus.left == x ? ClipEdge::left : ClipEdge::right;
#endif
        return;
    };
    auto addHorizontal = [curve, bounds, tStart, tEnd, focus, this  CLIP_PARAM(setBase)](float y) {
        if (!addingFill)
            return;
        float left = std::max(bounds.left, focus.left);
        float right = std::min(bounds.right, focus.right);
        if (left >= right) {
#if DEBUG_CLIP
           DebugClip* clipPtr = setBase();
           clipPtr->clipCorner = bounds.bottom <= focus.top
                   ? (bounds.right <= focus.left ? ClipCorner::topLeft : ClipCorner::topRight)
                   : (bounds.right <= focus.left ? ClipCorner::bottomLeft : ClipCorner::bottomRight);
#endif
            return;
        }
        if (curve.c.data->start.x > curve.c.data->end.x)
            std::swap(left, right);
        window->add(this, OpPtT({ left, y }, tStart), OpPtT({ right, y }, tEnd)  
                CLIP_PARAM(curve, tStart, tEnd));
#if DEBUG_CLIP
        DebugClip* clipPtr = setBase();
        clipPtr->clipEdge = focus.top == y ? ClipEdge::top : ClipEdge::bottom;
#endif
        return;
    };
    if (bounds.right <= focus.left)
        return addVertical(focus.left);
    if (bounds.left >= focus.right)
        return addVertical(focus.right);
    if (bounds.bottom <= focus.top)
        return addHorizontal(focus.top);
    if (bounds.top >= focus.bottom)
        return addHorizontal(focus.bottom);
    OpPtT startPtT = curve.ptTAtT(tStart);
    OpPtT endPtT = curve.ptTAtT(tEnd);
    if (focus.contains(bounds)) {
        window->add(curve, this, startPtT, endPtT, tStart, tEnd);
        return;
    }
    std::vector<OpPtT> ptTs;
    // !!! this generates parallels to window frame which are not part of original curve
    // bounds overlaps, but curve may not intersect; find interior ends, intersection with bounds
    auto addPin = [&ptTs, focus](OpPtT end) {
        OpPoint sect = { OpMath::PinSorted(focus.left, end.pt.x, focus.right),
                OpMath::PinSorted(focus.top, end.pt.y, focus.bottom) };
        ptTs.push_back({ sect, end.t });
    };
    auto addSects = [&curve, &ptTs](OpRoots roots, float xy, Axis axis) {
        for (float root : roots.roots) {
            OpPtT ptAtT = curve.ptTAtT(root);
        #if 0
            ptAtT.pt.choice(axis) = xy;
            addPin(ptAtT);
        #else  // !!! experiment with making frame intersection as accurate as possible for float
            ptTs.push_back(ptAtT);
        #endif
        }
    };
    addPin(startPtT);
    auto crossRoots = [&curve, &startPtT, &endPtT](Axis axis, float xy) {
        if (!OpMath::Between(startPtT.pt.choice(axis), xy, endPtT.pt.choice(axis)))
            return OpRoots();  // does not cross focus bounds
        return curve.axisRayHit(axis, xy, startPtT.t, endPtT.t);
    };
    addSects(crossRoots(Axis::vertical, focus.left), focus.left, Axis::vertical);
    addSects(crossRoots(Axis::horizontal, focus.top), focus.top, Axis::horizontal);
    addSects(crossRoots(Axis::vertical, focus.right), focus.right, Axis::vertical);
    addSects(crossRoots(Axis::horizontal, focus.bottom), focus.bottom, Axis::horizontal);
    addPin(endPtT);
    // for each span : if middle is inside focus, keep ends of span
    std::sort(ptTs.begin(), ptTs.end(), [](const auto& s1, const auto& s2) {
			return s1.t < s2.t; } );
    auto matchSides = [focus](const OpPtT& ptT) {
        int sides = 0;
        sides |= (focus.left >= ptT.pt.x) << 0;
        sides |= (focus.top >= ptT.pt.y) << 1;
        sides |= (focus.right <= ptT.pt.x) << 2;
        sides |= (focus.bottom <= ptT.pt.y) << 3;
        return sides;
    };
    const OpPtT* lastPtT = &ptTs.front();
    int lastSide = matchSides(*lastPtT);
    for (size_t index = 1; index < ptTs.size(); ++index) {
        const OpPtT& ptT = ptTs[index];
        int side = matchSides(ptT);
        if (lastPtT->t != ptT.t && lastPtT->pt != ptT.pt) {
            if (!(lastSide & side)) {
                window->add(curve, this, *lastPtT, ptT, tStart, tEnd);
    #if DEBUG_CLIP
                setSide(*lastPtT, ptT, lastSide, side);
    #endif
            } else if (addingFill) {
                OP_ASSERT(IDType::contour == opType.type || IDType::output == opType.type);
                window->add(this, *lastPtT, ptT  CLIP_PARAM(curve, tStart, tEnd));
    #if DEBUG_CLIP
                setSide(*lastPtT, ptT, lastSide, side);
    #endif
            }
        }
        lastPtT = &ptT;
        lastSide = side;
    }
}

void DebuggerAddPoly::add(const DebugOutput& debugOutput) {
    opType = OpType();
    opType.type = IDType::output;
    addingFill = true;
    add(debugOutput.curve, 0, 1);
}

void DebuggerAddPoly::add(const OpEdge* e) {
    opType = OpType(e);
    addingFill = false;
    if (debuggerState->showEdgeCurve)
        add(e->curve, 0, 1);
    else
        add(e->segment->c, e->startT, e->endT);
}

void DebuggerAddPoly::add(const OpSegment* s) {
    opType = OpType(s);
    addingFill = false;
    add(s->c, 0, 1);
}

void DebuggerAddPoly::add(const OpIntersection* i) {
    opType = OpType(i);
    addingFill = false;
    if (window->focus.contains(i->ptT.pt))
        window->add(this, i->ptT);
}

// !!! for segments making up area; color comes from contour
void DebuggerAddPoly::add(const OpContour* c) {
    opType = OpType(c, -1);
    addingFill = true;
    OpPoint last(SetToNaN::dummy);
    for (opType.curveIndex = 0; opType.curveIndex < (int) c->debugCurveData.size(); 
            ++opType.curveIndex) {
        std::vector<float> extrema;
        OpCurve opCurve(c->debugCurve(opType.curveIndex, &extrema), Rotated::yes);
        if (!opCurve.fullBounds().intersects(window->focus)) {
            add(opCurve, 0, 1);
            continue;
        }
        OpRoots tValues(0, 1);
        for (float ex : extrema) {
            tValues.add(ex);
        }
        tValues.sort();
        for (int index = 0; index < tValues.count() - 1; ++index) {
            add(opCurve, tValues.roots[index], tValues.roots[index + 1]);
        }   
   }
//    dmpPoly(this, 1);
//    OP_ASSERT(0);
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

OpPoint DebuggerPoly::callerEnd() const {
    return c.ptAtT(tEnd);
}

OpPoint DebuggerPoly::callerStart() const {
    return c.ptAtT(tStart);
}

std::string DebuggerPoly::debugDump(DebugLevel l, DebugBase b) const {
    std::string s;
    s += "local:" + STR(local.size()) + " ";
    if (IDType::edge == opType.type)
        s += "edge:";
    if (IDType::segment == opType.type)
        s += "segment:";
    if (IDType::intersection == opType.type)
        s += "intersection:";
    if (IDType::contour == opType.type)
        s += "contour:";
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
    for (OpDPoint pt : local) {
         s += pt.debugDump(defaultLevel, defaultBase) + "\n";
    }
    s.pop_back();
    return s;
}

void DebuggerPoly::dump() const {
    std::string s = debugDump(defaultLevel, defaultBase);
    OpDebugOut(s + "\n");
}

OpVector DebuggerPoly::normal(float t) const {
    return c.normal(tStart * (1 - t) + tEnd * t);
}

OpPoint DebuggerPoly::ptAtT(float t) const {
    return c.ptAtT(tStart * (1 - t) + tEnd * t);
}

OpVector DebuggerPoly::tangent(float t) const {
    return c.tangent(tStart * (1 - t) + tEnd * t);
}

#if OP_DEBUG_VALIDATE
void DebuggerPoly::validate() const {
    if (!isPrimary)
        return;
    if (c.context)
        OpDebugValidate(c);  // !!! add when needed
    opType.validate();
}
#endif

#if DEBUG_CLIP
std::string OpType::debugDump(DebugLevel, DebugBase) const {
    static std::array<std::string, 13> idNames { 
        "none", 
        "contour" ,
        "segment",
        "edge",
        "intersection",
        "coincident",
        "unsectable",  // intersection
        "unsectID",  // edge
        "distance",
        "pal",
        "tree",
        "limb",
        "output" 
    };
    std::string s;
    s += idNames[(int) type] + " ";
    s += "id:" + STR(id) + " ";
    if (IDType::contour == type)
        s += "(" + STR(curveIndex) + " of " + STR(contour->debugCurveData.size()) + ") ";
    if (inCcStorage)
        s += "inCcStorage ";
    if (drawn)
        s += "drawn ";
    s.pop_back();
    return s;
}

static void dumpOne(DebugClip& clip) {
    std::string s;
    s += "start:" + clip.start.debugDump(defaultLevel, defaultBase) + " ";
    if (clip.start != clip.clipStart && !clip.clipStart.debugIsUninitialized())
        s += "clip.start:" + clip.clipStart.debugDump(defaultLevel, defaultBase) + " ";
    s += "end:" + clip.end.debugDump(defaultLevel, defaultBase) + " ";
    if (clip.end != clip.clipEnd && !clip.clipEnd.debugIsUninitialized())
        s += "clip.end:" + clip.clipEnd.debugDump(defaultLevel, defaultBase) + " ";
    static std::array<std::string, 4> cornerNames { "topLeft", "topRight" ,
        "bottomLeft", "bottomRight" };
    if (ClipCorner::none != clip.clipCorner)
        s += "clipCorner:" + cornerNames[(int) clip.clipCorner - (int) ClipCorner::topLeft] + " ";
    static std::array<std::string, 4> edgeNames { "left", "top" ,
        "right", "bottom" };
    if (ClipEdge::none != clip.clipEdge)
        s += "clipEdge:" + edgeNames[(int) clip.clipEdge - (int) ClipEdge::left] + " ";
    auto sideNames = [](int side) {
        std::string s;
        for (int bit = 0; bit < 4; ++bit) {
            if ((1 << bit) & side) {
                s += edgeNames[bit] + " ";
            }
        }
        return s;
    };
    if (clip.lastSide > 0)
        s += "start side:" + sideNames(clip.lastSide);
    if (clip.side > 0)
        s += "start side:" + sideNames(clip.side);
    s.pop_back();
    OpDebugOut(s + "\n");
}

void dmpPoly(DebuggerWindow* window, int id) {
    OpDebugOut("focus: " + window->focus.debugDump(defaultLevel, defaultBase) + "\n");
    OpType opType;
    bool dumped = false;
    for (OpType& oType : window->debuggerState->ids) {
        if (oType.id != id)
            continue;
        dumped = true;
        opType = oType;
    }
    if (!dumped) {
        OpDebugOut("no match for:" + STR(id) + "\n");
        return;
    }
    int curveCount = IDType::contour == opType.type ? opType.contour->debugCurveData.size() : 1;
    for (opType.curveIndex = 0; opType.curveIndex < curveCount; ++opType.curveIndex) {
        std::vector<DebugClip*> debugClips;
        window->findDebugClips(opType, 0, 0, &debugClips);
        std::sort(debugClips.begin(), debugClips.end(), [](auto s1, auto s2) {
                return s1->clipStart.t < s2->clipStart.t; } );
        if (debugClips.empty()) {
            std::string s = "no debug clip for:" + STR(id);
            if (IDType::contour == opType.type)
                s +=  "/" + STR(opType.curveIndex);
            OpDebugOut(s + "\n");
        } else
            OpDebugOut(opType.debugDump(defaultLevel, defaultBase) + "\n");
        float t = 0;
        for (DebugClip* clip : debugClips) {
            if (clip->clipStart.t > t)
                OpDebugOut("gap: " + STR(t) + " to " + STR(clip->clipStart.t) + "\n");
            else if (clip->clipStart.t < t)
                OpDebugOut("overlap: " + STR(clip->clipStart.t) + " to " + STR(t) + "\n");
            dumpOne(*clip);
            t = clip->end.t;
        }
        if (1 != t)
            OpDebugOut("gap: " + STR(t) + " to 1\n");
    }
}

void dmpPoly(DebuggerAddPoly* addPoly, int id) {
    dmpPoly(addPoly->window, id);
}

void dmpPoly(DebuggerState* state, int id) {
    dmpPoly(&state->pictureWindow, id);
}

#endif