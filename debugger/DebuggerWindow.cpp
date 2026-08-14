// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpCurveCurve.h"
#include "DebuggerState.h"
#include <SDL3/SDL_error.h>

const std::array<std::string, 6> WheelTargetVerbage {
    "none",
    "pan",
    "zoom",
    "tune threshold",
    "scale font",
    "scroll text"
};

DebuggerWindow::DebuggerWindow(DebuggerState* state, WheelTarget target) 
    : debuggerState(state)
    , wheelTarget(target) {
    addPoly.debuggerState = state;
    addPoly.window = this;
}


#define STRD(x) OpDebugDStr(x, debugPrecision)

std::string OpDebugDStr(double value, int precision) {
    if (OpMath::IsNaN(value))
        return "NaN";
    if (!OpMath::IsFinite(value))
        return value > 0 ? "Inf" : "-Inf";
    if (0 == value)
        return "0";
    if (-1 == precision) {  // special hack for device 1/4 pixel measures
        int dev = (int) (value * 8);
        int frac = dev - dev / 8 * 8;
        if (frac < 0)
            frac = 8 - frac;
        std::string s = STR(dev / 8);
        int quarter = (frac + 1) / 2;
        s += 1 == quarter ? ".25" : 2 == quarter ? ".5" : 3 == quarter ? ".75" : "";
        if (dev != value * 8)
            s += frac & 1 ? "-" : "+";
        return s;
    }
    if (fabs(value) >= 0.001)
        return std::to_string(value);
    std::string s(16, '\0');
    auto written = std::snprintf(&s[0], s.size(), "%.*g", precision, value);
    s.resize(written);
    return s;
}

std::string OpDVector::debugDump(DebugLevel , DebugBase) const {
    return "{"  + STRD(dx) + ", " + STRD(dy) + "}";
}

void OpDVector::dump() const {
    OpDebugOut(debugDump(defaultLevel, defaultBase) + "\n");
}

std::string OpDPoint::debugDump(DebugLevel , DebugBase) const {
    return "{"  + STRD(x) + ", " + STRD(y) + "}";
}

void OpDPoint::dump() const {
    OpDebugOut(debugDump(defaultLevel, defaultBase) + "\n");
}

#if 0

struct OpPtDT {
    OpPoint pt() const {
        return dPt.point();
    }

    std::string debugDump(DebugLevel l, DebugBase b) const {
        return dPt.debugDump(l, b) + " t:" + STRD(dT);
    }

    void dump() const {
        OpDebugOut(debugDump(defaultLevel, defaultBase) + "\n");
    }

    OpDPoint dPt;
    double dT;
};

bool OpMath::BetweenD(double a, double b, double c) {
    OP_DEBUG_CODE(bool classicResult = (a <= b && b <= c) || (a >= b && b >= c));
    OP_DEBUG_CODE(bool cleverResult = (a - b) * (c - b) <= 0);
    OP_ASSERT(classicResult == cleverResult || fabs(b) < OpEpsilon);
    return (a - b) * (c - b) <= 0;
}

struct LineDPts {
	bool ptOnLine(OpDPoint ctrlPt, double threshold, bool debugOut) const {
        if (!OpMath::BetweenD(pts[0].x, ctrlPt.x, pts[1].x))
            return false;
        if (!OpMath::BetweenD(pts[0].y, ctrlPt.y, pts[1].y))
            return false;
        OpDVector dxy = pts[1] - pts[0];
        if (OpMath::IsNaN(threshold))
            threshold = OpEpsilon;
        threshold *= dxy.length();
        OpDVector sxy = ctrlPt - pts[0];
        double nearStart = dxy.cross(sxy);
        OpDVector exy = pts[1] - ctrlPt;
        double nearEnd = dxy.cross(exy);
        if (debugOut) {
            OpDebugOut("threshold:" + STRD(threshold) 
                    + " nearStart:" + STRD(nearStart)
                    + " nearEnd:" + STRD(nearEnd) + "\n");
        }
        if (fabs(nearStart) > threshold)
            return false;
        if (fabs(nearEnd) > threshold)
            return false;
        return true;
    }

    void debugDevice(const DebuggerWindow& w, OpDPoint midPt) {
        OpDVector v = pts[1] - pts[0];
        OpDVector midV = midPt - pts[0];
        OpDPoint ctr;
        if (v.dx > v.dy)
            ctr = { midPt.x, pts[0].y + midV.dx / v.dx * v.dy };
        else
            ctr = { pts[0].x + midV.dy / v.dy * v.dx, midPt.y };
        auto DevStr = [w](OpDPoint p){
            OpDPoint dPt = w.toDevice(p);
            std::string s = "{" + OpDebugDStr(dPt.x, -1) + ", " + OpDebugDStr(dPt.y, -1) + "}";
            return s;
        };
        std::string s = "dev[0]" + DevStr(pts[0]) + " dev[1]" + DevStr(pts[1]);
        s += " mid" + DevStr(midPt) + " ctr" + DevStr(ctr);
        OpDebugOut(s + "\n");
    }


    std::string debugDump(DebugLevel l, DebugBase b) const {
        return "pts[0]" + pts[0].debugDump(l, b) + " pts[1]" + pts[1].debugDump(l, b);
    }

	std::array<OpDPoint, 2> pts;
};
#endif

// primary is chosen as center of start/end pt closest to focus center
void DebuggerWindow::add(const OpCurve& curve, DebuggerAddPoly* polyAdder,
        const OpPtT& ptTS, const OpPtT& ptTE, float cStart, float cEnd) {
    // if adding a contour lengthen existing poly it it matches and close the contour as well...
    OP_ASSERT(curve.c.context);
    std::vector<DebuggerPoly>& polys = findPolys(polyAdder->opType);
    DebuggerPoly* bestFocusDist = nullptr;
    polys.emplace_back();
    for (int index = (int) polys.size() - 2; index >= 0; --index) {
        DebuggerPoly* test = &polys[index];
        if (test->opType.id != polyAdder->opType.id)
            break;
        if (test->isPrimary) {
            bestFocusDist = test;
            break;
        }
    }
    DebuggerPoly& poly = polys.back();
    poly.c = curve;
    poly.opType = polyAdder->opType;
    OpRect bounds { ptTS.pt, ptTE.pt };
    bounds = bounds.offset(OpPoint(0, 0) - focus.center()); // orthogonal from edge bounds to focus center
    poly.focusDist = std::max(std::min(fabsf(bounds.left), fabsf(bounds.right)),
            std::min(fabsf(bounds.top), fabsf(bounds.bottom)));
    if (!bestFocusDist || poly.focusDist < bestFocusDist->focusDist) {
        if (bestFocusDist)
            bestFocusDist->isPrimary = false;
        poly.isPrimary = true;
    }
    poly.tStart = ptTS.t;
    poly.tEnd = ptTE.t;
#if DEBUG_CLIP
    debugClips.push_back({ poly.opType, curve.ptTAtT(cStart), curve.ptTAtT(cEnd) });
#endif
    OpDPoint cStartPt = curve.debugPtAtDT(cStart);
    OpDPoint cEndPt = curve.debugPtAtDT(cEnd);
    OpDPoint fStartPt = curve.ptAtT(cStart);
    OpDPoint fEndPt = curve.ptAtT(cEnd);
    OpDVector cStartV = fStartPt - cStartPt;
    OpDVector cEndV = fEndPt - cEndPt;
    // curve is fully inside focus; split it into lines
    auto biasDPt = [curve, &ptTS, &ptTE, &cStartV, &cEndV](float dT) {
        double tRange = ptTE.t - ptTS.t;
        OpDPoint pt = curve.debugPtAtDT(dT);
        double startBias = (ptTE.t - dT) / tRange;
        double endBias = (dT - ptTS.t) / tRange;
        pt += cStartV * startBias + cEndV * endBias;
        return pt;
    };
    OpDPoint start = ptTS.t == cStart ? curve.ptAtT(ptTS.t) : biasDPt(ptTS.t);
    OpDPoint end = ptTE.t == cEnd ?  curve.ptAtT(ptTE.t) : biasDPt(ptTE.t);
 //   start here;
    // make cStart, cEnd regular non-debug params
    // calc delta from float curve w/cStart, cEnd and double curve
    // find delta to apply to double vals from tStart to tEnd
    // pass if tStart or tEnd was computed from focus side intersection
            // if so, and start pt is inside focus, back up tStart until equal or outside

    // split curve until a piece is linear
    append(polys, start);
    if (curve.debugIsLine(ptTS.t, ptTE.t)) {
        append(polys, end);
        return;
    }
// use device bounds to find t step
    OpDPoint devStart = toDevice(start);
    OpDPoint devEnd = toDevice(end);
    OpDVector devV = devEnd - devStart;
    int devSteps = std::min((int) fabs(devV.dx), (int) fabs(devV.dy));
    devSteps = std::max(1, devSteps);
// but limit steps to number of descernable t values
    int tLo = (int) (ptTS.t / OpEpsilon);
    int tHi = (int) (ptTE.t / OpEpsilon);
    int tSteps = tHi - tLo;
    tSteps = std::max(1, tSteps);
    int steps = std::min(devSteps, tSteps);
    for (int step = 0; step <= steps; ++step) {
        double fStep = (double) step / (double) steps;
        double dT = ptTS.t * (1 - fStep) + ptTE.t * fStep;
        OpDPoint pt = biasDPt(dT);
        append(polys, pt);
    }
}

// span is between first and last points, but does not extend last point (unless, see below)
void DebuggerWindow::add(std::vector<OpPoint>& pts ) {
    if (pts.empty())
        return;
    OpPoint last = pts.front();
    for (OpPoint pt : pts) {
        if (last == pt)
            continue;
#if 0
        polyPoints.emplace_back();
        DebuggerPoly& back = polyPoints.back();
        back.cData.start = last;
        back.cData.end = pt;
        back.c = { (ContextPtr) context(), &back.cData, sizeof(back.cData), 0 }; 
        OP_ASSERT(back.c.context);
        OpCurve curve(back.c, Rotated::no);
        add(curve, nullptr);
#else
        addLine(last, pt);
#endif
        last = pt;
    }
    OP_DEBUG_VALIDATE_CODE(validate());
}

void DebuggerWindow::add(DebuggerAddPoly* polyAdder, const OpPtT& ptT) {
    OP_ASSERT(IDType::intersection == polyAdder->opType.type);
    intersections.emplace_back();
    DebuggerPoly& poly = intersections.back();
    poly.opType = polyAdder->opType;
    poly.tStart = ptT.t;
    poly.tEnd = ptT.t;
#if DEBUG_CLIP
    debugClips.push_back({polyAdder->opType});
    DebugClip& debugClip = debugClips.back();
    debugClip.clipStart = ptT;
#endif
}

// for fill only
// span is between points, but does not extend last point unless last point equals first point
void DebuggerWindow::add(DebuggerAddPoly* polyAdder, const OpPtT& ptT1, const OpPtT& ptT2
        CLIP_PARAM(const OpCurve& curve, float cStart, float cEnd)) {
    OP_ASSERT(IDType::contour == polyAdder->opType.type);
    if (ptT1.pt == ptT2.pt)
        return;
    std::vector<OpDPoint>* lines = nullptr;
    auto getLines = [this, polyAdder, &lines]() {
        contours.emplace_back();
        DebuggerPoly& poly = contours.back();
        poly.opType = polyAdder->opType;
        lines = &poly.local;
    };
    bool isAligned = false;
    bool collapsedAligned = false;
    if (contours.empty()) {
        getLines();
    } else {
        DebuggerPoly& last = contours.back();
        OpDPoint dPt1(ptT1.pt);
        if ((!last.local.empty() && last.local.back() != dPt1)
                || last.opType.id != polyAdder->opType.id) {
            getLines();
        } else {
            lines = &last.local;
            auto aligned = [](OpDPoint p0, OpDPoint p1, OpDPoint p2) {
                return (p0.x == p1.x && p1.x == p2.x) || (p0.y == p1.y && p1.y == p2.y);
            };
            if (lines->size() > 1) {
                OpDPoint dPt2(ptT2.pt);
                isAligned = aligned((&lines->back())[-1], dPt1, dPt2);
                if (isAligned) {
                    lines->pop_back();
                    collapsedAligned = !lines->empty() && lines->back() == dPt2;
                    if (collapsedAligned)
                        lines->pop_back();
                }
            }
        }
    }
#if DEBUG_CLIP
    debugClips.push_back({ polyAdder->opType, curve.ptTAtT(cStart), curve.ptTAtT(cEnd) });
#endif
    if (!isAligned && (lines->empty() || lines->back() != ptT1.pt))
        lines->push_back(ptT1.pt);
    if (!collapsedAligned)
        lines->push_back(ptT2.pt);
    OP_DEBUG_VALIDATE_CODE(validate());
}

DebuggerPoly& DebuggerWindow::add(const OpRect& r, uint32_t color, float thickness) {
    rects.emplace_back();
    DebuggerPoly& poly = rects.back();
    std::vector<OpPoint> points { 
        { r.left, r.top }, { r.left, r.bottom }, { r.right, r.bottom }, { r.right, r.top } };
    std::vector<OpPoint>& lines = poly.device;
    for (OpPoint pt : points) {
        lines.push_back(pt);
    }
    poly.contours.push_back(points.size());
    poly.color = color;
    poly.thickness = thickness;
    OP_DEBUG_VALIDATE_CODE(validate());
    return poly;
}

void DebuggerWindow::addLine(OpPoint pt1, OpPoint pt2) {
    DebuggerPoly& poly = lines.back();
    std::vector<OpPoint>& line = poly.device;
    line.push_back(pt1);
    line.push_back(pt2);
    poly.contours.push_back(2);
}

OpDebugText& DebuggerWindow::addClipped(std::string s , OpPoint pt, uint32_t color, TTF_Font* f) {
    OpDebugText& text = addText(s, pt, color, f);
    text.clip = true;
    return text;
}

OpDebugText& DebuggerWindow::addText(std::string s, OpPoint device, uint32_t color, TTF_Font* f, 
        bool rotated) {
    OpDebugText& text = texts.emplace_back();
    text.cacheIndex = addText(s, color, f);
    text.pt = device;
    text.debugLocal = device;
    text.vertical = rotated;
    return text;
}

// span is between this point and last point, if any
void DebuggerWindow::append(std::vector<DebuggerPoly>& polys, OpDPoint pt) {
    if (polys.empty())
        polys.emplace_back();
    polys.back().local.push_back(pt);
//    validate();  // !!! too soon, added poly isn't initialized
}

void DebuggerWindow::clearWindow() {
    allocateBuffers();
    focus = OpRect();
    edges.clear();
    contours.clear();
    intersections.clear();
    segments.clear();
    output.clear();
    rects.clear();
    polyPoints.clear();
    lines.clear();
    deleteTextCache();
    texts.clear();
    points.clear();
#if DEBUG_CLIP
    debugClips.clear();
#endif
}

OpContext* DebuggerWindow::context() {
    return debuggerState->context;
}

std::string DebuggerWindow::debugDump(DebugLevel l, DebugBase b) const{
    std::string s;
    auto addToDump = [&s, l, b](std::string name, const std::vector<DebuggerPoly>& polys) {
        for (const DebuggerPoly& poly : polys) {
            s += poly.debugDump(l, b);
        }
    };
    addToDump("contours", contours);
    addToDump("edges", edges);
    addToDump("intersections", intersections);
    addToDump("segments", segments);
    addToDump("rects", rects);
    addToDump("polyPoints", polyPoints);
    addToDump("lines", lines);
    s += "texts:\n";
    for (const OpDebugText& text : texts)
        s += text.debugDump(l, b);
    return s;
}

DebuggerPoly* DebuggerWindow::findPolyByID(int id) {
    DebuggerPoly* result = nullptr;
    auto findPoly = [id, &result](std::vector<DebuggerPoly>* polys) {
        for (DebuggerPoly& poly : *polys) {
            if (poly.opType.id == id)
                result = &poly;
        }
    };
    for (std::vector<DebuggerPoly>* polys : polyIDs)
        findPoly(polys);
    return result;
}

DebuggerPoly* DebuggerWindow::findRectByID(int id) {
    DebuggerPoly* result = nullptr;
    for (DebuggerPoly& poly : rects) {
        if (poly.opType.id == id)
            result = &poly;
    }
    return result;
}

std::vector<DebuggerPoly>& DebuggerWindow::findPolys(OpType opType) {
    switch (opType.type) {
        case IDType::edge:
            return edges;
       case IDType::contour:
           return contours;
       case IDType::segment:
           return segments;
       case IDType::intersection:
            return intersections;
       case IDType::output:
           return output;
       default:
           OP_ASSERT(0);
    }
    return edges;
}

const NativeTextCache& DebuggerWindow::getCache(size_t index) const {
    OP_ASSERT(index < textCache.size());
    return textCache[index];
}

void DebuggerWindow::playbackCommon(const char*& str) {
    DEBUG_SET_COMMON_STRUCT(screen);
    if (OpDebugOptional(str, "windowWidth")) {
        int windowWidth = (int) OpDebugReadSizeT(str);
        OpDebugRequired(str, "windowHeight");
        int windowHeight = (int) OpDebugReadSizeT(str);
        SDL_SetWindowSize(window, windowWidth, windowHeight);
    }
    if (OpDebugOptional(str, "windowX")) {
        int windowX = (int) OpDebugReadSizeT(str);
        OpDebugRequired(str, "windowY");
        int windowY = (int) OpDebugReadSizeT(str);
        SDL_SetWindowPosition(window, windowX, windowY);
    }
    if (OpDebugOptional(str, "windowVisible"))
        SDL_ShowWindow(window);
    scale = OpDebugReadNamedFloat(str, "scale");  // factor to go from local to device (zero is uninitialized)
    OpDebugRequired(str, "fontSize");
    fontSize = (int) OpDebugReadSizeT(str);
}


std::string DebuggerWindow::recordCommon() {
    std::string s;
    DebugLevel l = DebugLevel::file;
    DebugBase b = DebugBase::hex;
    DEBUG_DUMP_COMMON_STRUCT(screen);
    int windowWidth, windowHeight;
    if (SDL_GetWindowSize(window, &windowWidth, &windowHeight)) {
        s += "windowWidth:" + STR(windowWidth) + " ";
        s += "windowHeight:" + STR(windowHeight) + " ";
    }
    int windowX, windowY;
    if (SDL_GetWindowPosition(window, &windowX, &windowY)) {
        s += "windowX:" + STR(windowX) + " ";
        s += "windowY:" + STR(windowY) + " ";
    }
    if (0 == (SDL_GetWindowFlags(window) & SDL_WINDOW_HIDDEN))
        s += "windowVisible ";
    if (!OpMath::IsDebugNaN((float) scale))
        s += debugValue(DebugLevel::error, b, "scale", (float) scale) + " ";
    s += "fontSize:" + STR(fontSize) + " ";
    return s;
}

void DebuggerWindow::setSize() {
    int x, y;
    if (!SDL_GetWindowSize(window, &x, &y)) {
        OpDebugOut("Couldn't get window " + name + " size: " + std::string(SDL_GetError()) + "\n");
        return;
    }
    screen = OpRect(0, 0, (float) x, (float) y);
    if (!focus.isFinite())
        focus = screen;
    allocateBuffers();
}

std::vector<std::vector<DebuggerPoly>*> DebuggerWindow::tangentPolys() {
    std::vector<std::vector<DebuggerPoly>*> result;
    if (!debuggerState->hideEdges)
        result.push_back(&edges);
    if (debuggerState->showSegments)
        result.push_back(&segments);
    if (debuggerState->showContours)
        result.push_back(&contours);
    return result;
}

OpPoint DebuggerWindow::toLocal(OpPoint pt) const {
    return { (float) (pt.x / scale + focus.left), (float) (pt.y / scale + focus.top) };
}

OpDPoint DebuggerWindow::toLocal(OpDPoint pt) const {
    return { pt.x / scale + focus.left, pt.y / scale + focus.top };
}

// return local space point in device space
OpPoint DebuggerWindow::toDevice(OpPoint pt) const {
    return { (float) ((pt.x - focus.left) * scale), (float) ((pt.y - focus.top) * scale) };
}

OpDPoint DebuggerWindow::toDevice(OpDPoint pt) const {
    return { (pt.x - focus.left) * scale, (pt.y - focus.top) * scale };
}

#if OP_DEBUG
std::string DebuggerWindow::debugTextDump(size_t index) {
    OP_ASSERT(index < textCache.size());
    return textCache[index].debugDump();
}
#endif

#if OP_DEBUG_VALIDATE
void DebuggerWindow::validate() const {
for (std::vector<DebuggerPoly>& polys : polyIDs) {
        for (const DebuggerPoly& poly : polys) {
            poly.validate();
        }
    }
}
#endif

#if DEBUG_CLIP
void DebuggerWindow::findDebugClips(OpType& opType, float tStart, float tEnd,
            std::vector<DebugClip*>* result) {
    for (DebugClip& debugClip : debugClips) {
        if (debugClip.opType.type != opType.type)
            continue;
        if (debugClip.opType.id != opType.id)
            continue;
        if (debugClip.opType.curveIndex != opType.curveIndex)
            continue;
        if (tStart != tEnd && (debugClip.start.t != tStart || debugClip.end.t != tEnd))
            continue;
        result->push_back(&debugClip);
    }
}

DebugClip* DebuggerWindow::createDebugClip(OpType& opType,
        const OpCurve& curve, float tStart, float tEnd) {
    std::vector<DebugClip*> result;
    findDebugClips(opType, tStart, tEnd, &result);
    for (DebugClip* debugClip : result) {
        if (debugClip->start.t == tStart && debugClip->end.t == tEnd)
            return debugClip;
    }
    debugClips.push_back({ opType, curve.ptTAtT(tStart), curve.ptTAtT(tEnd) });
    DebugClip* debugClip = &debugClips.back();
    debugClip->clippedOut = true;
    return debugClip;
}
#endif

DebuggerEvent::DebuggerEvent(DebuggerState* debuggerState, SDL_Keymod mod, SDL_WindowID windowID) {
    keyMods = KeyMods::none;
    if (SDL_KMOD_SHIFT & mod)
        keyMods |= KeyMods::shift;
    if (SDL_KMOD_CTRL & mod)
        keyMods |= KeyMods::ctrl;
    if (SDL_KMOD_ALT & mod)
        keyMods |= KeyMods::alt;
    if (windowID)
        focused = debuggerState->focus(windowID);
}

DrawLevel DebuggerEvent::doEvent() {
    if (!focused)
        return DrawLevel::none;
    return focused->event(*this);
}

int DebuggerEvent::KeyModMultiplier(KeyMods mods) {
    int scale = 1;
    if (KeyMods::shift == (KeyMods::shift & mods))
        scale = 2;
    if (KeyMods::ctrl == (KeyMods::ctrl & mods))
        scale *= 4;
    if (KeyMods::alt == (KeyMods::alt & mods))
        scale *= 16;
    return scale;
}
