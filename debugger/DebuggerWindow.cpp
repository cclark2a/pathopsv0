// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpContext.h"
#include "OpDebugPicture.h"
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>

int keyModMultiplier(KeyMods mods) {
    int scale = 1;
    if (KeyMods::shift == (KeyMods::shift & mods))
        scale = 2;
    if (KeyMods::ctrl == (KeyMods::ctrl & mods))
        scale *= 4;
    if (KeyMods::alt == (KeyMods::alt & mods))
        scale *= 16;
    return scale;
}

DebuggerEvent::DebuggerEvent(DebuggerState* debuggerState, SDL_Keymod mod, SDL_WindowID windowID) {
    keyMods = KeyMods::none;
    if (SDL_KMOD_SHIFT & mod)
        keyMods |= KeyMods::shift;
    if (SDL_KMOD_CTRL & mod)
        keyMods |= KeyMods::ctrl;
    if (SDL_KMOD_ALT & mod)
        keyMods |= KeyMods::alt;
    focused = debuggerState->focus(windowID);
}

Window::Window() {
    addPoly.window = this;
}

struct OpContextSaveThreshold {
    OpContextSaveThreshold(OpContext* c, OpVector threshold) {
        context = c;
        save = context->aliases.threshold;
        context->aliases.threshold = threshold;
        context->aliases.thresholdLength = context->aliases.threshold.length();
    }

    ~OpContextSaveThreshold() {
        context->aliases.threshold = save;
        context->aliases.thresholdLength = context->aliases.threshold.length();
    }

    OpContext* context;
    OpVector save;
};

void Window::add(const OpCurve& curve, DebuggerAddPoly* polyAdder) {
        // if adding a contour lengthen existing poly it it matches and close the contour as well...
    if (!polyAdder->continueCurve) {
        polys.emplace_back();
        polys.back().c = curve.c;
        if (polyAdder->contour)
            polys.back().color = polyAdder->contour->debugColor;
    }
    DebuggerPoly& poly = polys.back();
    if (polyAdder) {
        poly.edge = polyAdder->edge;
        poly.segment = polyAdder->segment;
        poly.contour = polyAdder->contour;
        poly.isPrimary = true;
        if (poly.contour)
            poly.thickness = DebuggerPoly::fill_thickness;
    }
    OpContextSaveThreshold save(context(), threshold);
    // curve is fully inside focus; split it into lines
    // lengthen curve while longer is linear
    float start = 0;
    float end = 1;
    // split curve until a piece is linear
    append(curve.c.data->start);
    OpCurve piece = curve;
    do {
        while (!piece.isLine()) {
            end = OpMath::Average(start, end);
            piece = curve.subDivide(start, end);
        }
        append(piece.c.data->end);
        float span = end - start;
        OP_ASSERT(span > 0);
        // lengthen the curve while a piece is linear (and while it can be longer)
        bool pieceIsLine = false;
        OpPoint lastEnd(SetToNaN::dummy);
        do {
            start = end;
            end = std::min(1.f, start + span);
            if (start >= end)
                return;
            piece = curve.subDivide(start, end);
            pieceIsLine = piece.isLine();
            if (!pieceIsLine && lastEnd.isFinite()) {
                append(lastEnd);
                start = OpMath::Average(start, end);
                piece = curve.subDivide(start, end);
                break;
            }
            span *= 2;
            lastEnd = piece.c.data->end;
        } while (pieceIsLine && end < 1.f);
    } while (end < 1 || !piece.isLine());
    append(curve.c.data->end);
}

// span is between first and last points, but does not extend last point (unless, see below)
void Window::add(std::vector<OpPoint>& pts ) {
    if (pts.empty())
        return;
    OpPoint last = pts.front();
    for (OpPoint pt : pts) {
        if (last == pt)
            continue;
        polys.emplace_back();
        DebuggerPoly& back = polys.back();
        back.cData.start = last;
        back.cData.end = pt;
        back.c = { (ContextPtr) context(), &back.cData, sizeof(back.cData), 0 }; 
        OpCurve curve(back.c, Rotated::no);
        add(curve, nullptr);
        last = pt;
    }
}

// for fill only
// span is between points, but does not extend last point unless last point equals first point
void Window::add(OpPoint pt1, OpPoint pt2, DebuggerAddPoly* polyAdder) {
    if (pt1 == pt2)
        return;
    std::vector<OpPoint>* lines = nullptr;
    auto getLines = [this, polyAdder, &lines]() {
        polys.emplace_back();
        DebuggerPoly& poly = polys.back();
        poly.edge = polyAdder->edge;
        poly.segment = polyAdder->segment;
        poly.contour = polyAdder->contour;
        lines = &poly.local;
    };
    if (polys.empty()) {
        getLines();
    } else {
        DebuggerPoly& last = polys.back();
        if (!last.local.empty() && last.local.back() != pt1) {
            getLines();
        } else {
            lines = &last.local;
            OP_ASSERT(lines->size() > 1);
            auto aligned = [](OpPoint p0, OpPoint p1, OpPoint p2) {
                return (p0.x == p1.x && p1.x == p2.x) || (p0.y == p1.y && p1.y == p2.y);
            };
            if (aligned((&lines->back())[-1], pt1, pt2))
                lines->pop_back();
        }
    }
    if (lines->empty() || lines->back() != pt1)
        lines->push_back(pt1);
    lines->push_back(pt2);
}

DebuggerPoly& Window::add(const OpRect& r, uint32_t color, float thickness) {
    polys.emplace_back();
    DebuggerPoly& poly = polys.back();
    std::vector<OpPoint> points { 
        { r.left, r.top }, { r.left, r.bottom }, { r.right, r.bottom }, { r.right, r.top } };
    std::vector<OpPoint>& lines = poly.device;
    for (OpPoint pt : points) {
        lines.push_back(pt);
    }
    poly.contours.push_back(points.size());
    poly.color = color;
    poly.thickness = thickness;
    return poly;
}

void Window::addLine(OpPoint pt1, OpPoint pt2) {
    DebuggerPoly& poly = polys.back();
    std::vector<OpPoint>& lines = poly.device;
    lines.push_back(pt1);
    lines.push_back(pt2);
    poly.contours.push_back(2);
}

OpDebugText& Window::addText(std::string s, OpPoint device, uint32_t color, bool rotated) {
    OpDebugText& text = texts.emplace_back();
    text.cacheIndex = addText(s, color);
    text.pt = device;
    text.debugLocal = device;
    text.vertical = rotated;
    return text;
}

void Window::clearWindow() {
    focus = OpRect();
    polys.clear();
    texts.clear();
    points.clear();
}

OpContext* Window::context() {
    return debuggerState->context;
}

DebuggerPoly* Window::findPoly(const OpEdge* edge) {
    for (DebuggerPoly& poly : polys) {
        if (poly.edge == edge)
            return &poly;
    }
    OP_ASSERT(0);
    return nullptr;
}

DebuggerPoly* Window::findPoly(const OpSegment* segment) {
    for (DebuggerPoly& poly : polys) {
        if (poly.segment == segment)
            return &poly;
    }
    OP_ASSERT(0);
    return nullptr;
}

const NativeTextCache& Window::getCache(size_t index) {
    OP_ASSERT(index < textCache.size());
    return textCache[index];
}

void Window::setSize() {
    int x, y;
    if (!SDL_GetWindowSize(window, &x, &y)) {
        OpDebugOut("Couldn't get window " + name + " size: " + std::string(SDL_GetError()) + "\n");
        return;
    }
    screen = OpRect(0, 0, x, y);
    if (!focus.isFinite())
        focus = screen;
    allocateBuffers(x, y);
}

#if OP_DEBUG_DUMP
std::string Window::debugTextDump(size_t index) {
    OP_ASSERT(index < textCache.size());
    return textCache[index].debugDump();
}
#endif

DebuggerState::DebuggerState() {
#if 1
    opFileName = "d:/gerrit/skia/out/Debug/obj/dmp.txt";
#else
    opFileName = "c:/users/cclar/source/repos/v0/v0/dmp2.txt";
#endif
    pictureWindow.debuggerState = this;
    textWindow.debuggerState = this;
    SDL_AppResult result = textWindow.init("text", { -100, -100 });
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise text window: " + std::string(SDL_GetError()) + "\n");
    }
    result = pictureWindow.init("picture", { 100, 100 } );
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise picture window: " + std::string(SDL_GetError()) + "\n");
    }
}

DrawLevel DebuggerEvent::doEvent() {
    if (!focused)
        return DrawLevel::none;
    return focused->event(*this);
}

void DebuggerState::draw() {
    pictureWindow.draw();
    textWindow.draw();
}

Window* DebuggerState::focus(SDL_WindowID id) {
    return pictureWindow.windowID == id ? (Window*) &pictureWindow :
            textWindow.windowID == id ? (Window*) &textWindow : nullptr;
}

void DebuggerState::redraw() {
    if (!context) 
        return;
    pictureWindow.redraw();
    textWindow.redraw();
    draw();
}

void DebuggerState::update() {
    delete context;
    context = fromFile(opFileName);
    debugGlobalContext = context;
    redraw();
}
