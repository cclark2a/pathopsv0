// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpCurveCurve.h"
#include "OpDebugPicture.h"
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>

Window::Window(DebuggerState* state) 
    : debuggerState(state) {
    addPoly.debuggerState = state;
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
        if (IDType::contour == polyAdder->opType.idType)
            polys.back().color = polyAdder->opType.contour->debugColor;
    }
    DebuggerPoly& poly = polys.back();
    if (polyAdder) {
        poly.opType = polyAdder->opType;
        poly.isPrimary = true;
        if (IDType::contour == poly.opType.idType)
            poly.thickness = DebuggerPoly::fill_thickness;
    }
    OpContextSaveThreshold save(context(), debuggerState->threshold);
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
        poly.opType = polyAdder->opType;
        lines = &poly.local;
    };
    if (polys.empty()) {
        getLines();
    } else {
        DebuggerPoly& last = polys.back();
        if ((!last.local.empty() && last.local.back() != pt1)
                || last.opType.edge != polyAdder->opType.edge) {  // compare any union pointer...
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

OpDebugText& Window::addText(std::string s, OpPoint device, uint32_t color, TTF_Font* f, 
        bool rotated) {
    OpDebugText& text = texts.emplace_back();
    text.cacheIndex = addText(s, color, f);
    text.pt = device;
    text.debugLocal = device;
    text.vertical = rotated;
    return text;
}

void Window::clearWindow() {
    focus = OpRect();
    polys.clear();
    deleteTextCache();
    texts.clear();
    points.clear();
}

OpContext* Window::context() {
    return debuggerState->context;
}

DebuggerPoly* Window::findPolyByID(int id) {
    for (DebuggerPoly& poly : polys) {
        if (poly.opType.id == id)
            return &poly;
    }
    return nullptr;
}

const NativeTextCache& Window::getCache(size_t index) {
    OP_ASSERT(index < textCache.size());
    return textCache[index];
}

void Window::playbackCommon(const char*& str) {
    DEBUG_SET_COMMON_STRUCT(screen);
    if (OpDebugOptional(str, "windowWidth")) {
        int windowWidth = OpDebugReadSizeT(str);
        OpDebugRequired(str, "windowHeight");
        int windowHeight = OpDebugReadSizeT(str);
        SDL_SetWindowSize(window, windowWidth, windowHeight);
    }
    if (OpDebugOptional(str, "windowX")) {
        int windowX = OpDebugReadSizeT(str);
        OpDebugRequired(str, "windowY");
        int windowY = OpDebugReadSizeT(str);
        SDL_SetWindowPosition(window, windowX, windowY);
    }
    if (OpDebugOptional(str, "windowVisible"))
        SDL_ShowWindow(window);
}


std::string Window::recordCommon() {
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
    return s;
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

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel HelpWindow::event(const DebuggerEvent& event) {
    OP_ASSERT(debuggerState->lastFocus);
    return debuggerState->lastFocus->event(event);
}

// updated when picture and text window change focus
void HelpWindow::redraw() {
    clearWindow();
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    OpPoint localLocation(10, 10);
    PictureWindow& picWin = debuggerState->pictureWindow;
    TextWindow& textWin = debuggerState->textWindow;
    auto nextLine = [&localLocation, detailFont, this](std::string str) {
        OpDebugText& text = addText(str, localLocation, debugBlack, detailFont);
        const NativeTextCache& cache = getCache(text.cacheIndex);
        localLocation.y += cache.size.dy;
    };
    auto showHide = [nextLine](const char* prefix, bool bit, const char* postfix) {
        nextLine(prefix + STR(bit ? "  hide " : "  show ") + postfix);
    };
    std::string thres = STR(debuggerState->thresholdWheel) + " / "
        + STR(debuggerState->thresholdMultiplier) + " ";
    std::string depthInfo;
    if (OpCurveCurve* curveCurve = context()->debugCurveCurve)
        depthInfo = ": " + STR(debuggerState->depth) + " / " + STR(curveCurve->depth);
    if (&picWin == debuggerState->lastFocus) {
        nextLine("\xE2\x86\x90 \xE2\x86\x91 \xE2\x86\x92 \xE2\x86\x93 "
                "pan picture left/up/right/down");
        showHide("c", picWin.drawCentersOn, "center points");
        showHide("C", debuggerState->drawContoursOn, "contours");
        nextLine("d / D  curve/curve intersection depth" + depthInfo);
        showHide("e", debuggerState->drawEdgesOn, "edges");
        showHide("E", debuggerState->drawEpsilonOn, "epsilon");
        showHide("f", picWin.drawFillOn, "fill");
        nextLine("g  show grid");
        showHide("h", picWin.drawHullsOn, "hulls");
        showHide("H", picWin.drawEdgeHullsOn, "edge hull intersection points");
        showHide("i", picWin.drawIDsOn, "show IDs");
        showHide("I", debuggerState->drawIntersectionsOn, "intersections");
        showHide("k", picWin.drawControlsOn, "controls");
        showHide("p", picWin.drawPointsOn, "points");
        nextLine("P  playback");
        nextLine("R  record");
        showHide("s", debuggerState->drawSegmentsOn, "segments");
        showHide("t", picWin.drawTangentsOn, "tangents");
        showHide("T", picWin.drawTsOn, "t values");
        showHide("v", picWin.drawValuesOn, "values");
        showHide("w", picWin.drawWindingsOn, "windings");
        showHide("x", debuggerState->drawHexOn, "hex");
        nextLine("0-9  precision: " + STR(debugPrecision));
        nextLine("~  tune threshold multiplier: " + thres);
        nextLine("?  hide help");
    } else {
        OP_ASSERT(&textWin == debuggerState->lastFocus);
        showHide("a", textWin.showAll, "all");
        showHide("A", textWin.showAliases, "aliases");
        showHide("C", debuggerState->drawContoursOn, "contours");
        nextLine("d / D  curve/curve intersection depth" + depthInfo);
        showHide("e", debuggerState->drawEdgesOn, "edges");
        showHide("f", textWin.showFull, "full relationship");
        showHide("h", textWin.showEdgeHulls, "edge hulls");
        showHide("l", textWin.showLinks, "links");
        showHide("p", textWin.showPoints, "points");
        showHide("r", textWin.showRays, "rays");
        showHide("s", debuggerState->drawSegmentsOn, "segments");
        showHide("t", textWin.showTree, "tree");
        showHide("x", debuggerState->drawHexOn, "hex");
        nextLine("0-9  precision: " + STR(debugPrecision));
        nextLine("~  tune threshold multiplier: " + thres);
        nextLine("?  hide help");
    }
}

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

DebuggerState::DebuggerState() 
    : pictureWindow(this)
    , textWindow(this)
    , helpWindow(this) {
#if 1
    opFileName = "d:/gerrit/skia/out/Debug/obj/dmp.txt";
#else
    opFileName = "c:/users/cclar/source/repos/v0/v0/dmp2.txt";
#endif
    if (SDL_APP_CONTINUE != (error = pictureWindow.addFont(14)))
        OpDebugOut("Couldn't add picture font: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (error = textWindow.addFont(14)))
        OpDebugOut("Couldn't add text font: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (error = textWindow.addFont(18, &textWindow.detailFont)))
        OpDebugOut("Couldn't add text detail font: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (error = textWindow.init("text", { -100, -100 })))
        OpDebugOut("Couldn't initialise text window: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (error = pictureWindow.init("picture", { 100, 100 } )))
        OpDebugOut("Couldn't initialise picture window: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (error = helpWindow.init("help", { -200, -200 })))
        OpDebugOut("Couldn't initialise help window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(helpWindow.window);
}

void DebuggerState::draw() {
    debugEpsilon = drawEpsilonOn;   // !!! eventually, merge so duplication is unnecessary
    pictureWindow.draw();
    textWindow.draw();
    helpWindow.redraw();
    helpWindow.draw();
}

DrawLevel DebuggerState::eventCommon(const DebuggerEvent& debuggerEvent) {
    if (debuggerEvent.wheel && tuneThreshold) {
        int scale = DebuggerEvent::KeyModMultiplier(debuggerEvent.keyMods);
        thresholdWheel -= debuggerEvent.wheel * scale;
        thresholdMultiplier = powf(2, thresholdWheel / 32.f);
        return DrawLevel::update;
    }
    uint8_t key = debuggerEvent.key;
    switch (key) {
        case 'C':
            drawContoursOn ^= true;
            break;
        case 'd':
            setDepth(++depth);
            break;
        case 'D':
            setDepth(--depth);
            break;
        case 'e':
            drawEdgesOn ^= true;
            break;
        case 'I':
            drawIntersectionsOn ^= true;
            break;
        case 'P':
            playback();
            break;
        case 'R':
            record();
            break;
        case 's':
            drawSegmentsOn ^= true;
            break;
        case 'x':
            drawHexOn ^= true;
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            debugPrecision = key - '0';
            break;
        case '-':
            debugPrecision = -1;
            break;
        case '~':
            tuneThreshold ^= true;
            break;
        case '?':
            drawHelp ^= true;
            if (drawHelp)
                SDL_ShowWindow(helpWindow.window);
            else
                SDL_HideWindow(helpWindow.window);
            break;
        default:
            return DrawLevel::none;
    }
    return key ? DrawLevel::update : DrawLevel::none;
}

std::string DebuggerState::floatToStr(float f) {
    return drawHexOn ? OpDebugDumpHex(f) : STR(f);
}

Window* DebuggerState::focus(SDL_WindowID id) {
    if (helpWindow.windowID == id)
        return lastFocus;
    lastFocus = pictureWindow.windowID == id ? (Window*) &pictureWindow :
            textWindow.windowID == id ? (Window*) &textWindow : nullptr;
//    OP_ASSERT(lastFocus);  // if left running, id may not match any window
    return lastFocus;
}

static std::string fileToStr(std::string filename) {
    std::string buffer;
    FILE* file = fopen(filename.c_str(), "r");
    OP_ASSERT(file);
    int seek = fseek(file, 0, SEEK_END);
    OP_ASSERT(!seek);
    long size = ftell(file);
    fclose(file);
    file = fopen(filename.c_str(), "r");
    buffer.resize(size);
    fread(&buffer[0], 1, size, file);
    fclose(file);
    return buffer;
}

void DebuggerState::playback() {
    std::string buffer = fileToStr("DebuggerState.txt");
    if (buffer.empty())
        return;
    const char* str = buffer.c_str();
    while (OpDebugOptional(str, "id")) {
        int id = OpDebugReadSizeT(str);
        auto foundID = std::find_if(ids.begin(), ids.end(), [id](const OpType& opType) {
                return id == opType.id; });
        if (ids.end() != foundID)
            foundID->selected = true;
    }
    // !!! add any additional global state here
    DEBUG_SET_STRUCT(helpWindow, threshold);
    DEBUG_SET_FLOAT(threshold, thresholdMultiplier);
    DEBUG_SET_REQUIRED_VALUE(thresholdMultiplier, thresholdWheel);
    DEBUG_SET_REQUIRED_VALUE(thresholdWheel, depth);
    DEBUG_SET_REQUIRED_VALUE(depth, error);
    DEBUG_SET_BOOL(error, drawContoursOn);
    DEBUG_SET_BOOL(drawContoursOn, drawEdgesOn);
    DEBUG_SET_BOOL(drawEdgesOn, drawEpsilonOn);
    DEBUG_SET_BOOL(drawEpsilonOn, drawHexOn);
    DEBUG_SET_BOOL(drawHexOn, drawIntersectionsOn);
    DEBUG_SET_BOOL(drawIntersectionsOn, drawSegmentsOn);
    DEBUG_SET_BOOL(drawSegmentsOn, tuneThreshold);
    DEBUG_SET_BOOL(tuneThreshold, drawHelp);
    pictureWindow.playback(str);
    textWindow.playback(str);
    helpWindow.playback(str);
}

void DebuggerState::record() {
#if 01 && defined _WIN32
   char full[_MAX_PATH];
   if( _fullpath( full, ".\\", _MAX_PATH ) != NULL )
      OpDebugOut( "Full path is: %s" + std::string(full) + "\n");
   else
      OpDebugOut( "Invalid path\n" );
#endif
    std::string s;
    for (auto& id : ids) {
        if (id.selected)
            s += "id:" + STR(id.id) + " ";
    }
    if (!s.empty())
        s.back() = '\n';
    // !!! add any additional global state here
    DebugLevel l = DebugLevel::file;
    DebugBase b = DebugBase::hex;
    DEBUG_DUMP_STRUCT(helpWindow, threshold);
    DEBUG_DUMP_FLOAT(threshold, thresholdMultiplier);
    DEBUG_DUMP_REQUIRED_VALUE(thresholdMultiplier, thresholdWheel);
    DEBUG_DUMP_REQUIRED_VALUE(thresholdWheel, depth);
    DEBUG_DUMP_REQUIRED_VALUE(depth, error);
    DEBUG_DUMP_BOOL(error, drawContoursOn);
    DEBUG_DUMP_BOOL(drawContoursOn, drawEdgesOn);
    DEBUG_DUMP_BOOL(drawEdgesOn, drawEpsilonOn);
    DEBUG_DUMP_BOOL(drawEpsilonOn, drawHexOn);
    DEBUG_DUMP_BOOL(drawHexOn, drawIntersectionsOn);
    DEBUG_DUMP_BOOL(drawIntersectionsOn, drawSegmentsOn);
    DEBUG_DUMP_BOOL(drawSegmentsOn, tuneThreshold);
    DEBUG_DUMP_BOOL(tuneThreshold, drawHelp);
    s += pictureWindow.record();
    s += textWindow.record();
    s += helpWindow.record();
	FILE* file = fopen("DebuggerState.txt", "w");
    fwrite(&s[0], 1, s.size(), file);
	fclose(file);
}

void DebuggerState::redraw() {
    if (!context) 
        return;
    pictureWindow.redraw();
    textWindow.redraw();
    helpWindow.redraw();
    draw();
}

// -1: draw none ; 0: draw all ; > 0 draw matching depth
void DebuggerState::setDepth(int ) {
    int maxDepth = 0;
	for (auto& id : ids) {
        if (IDType::edge != id.idType)
            continue;
        if (id.edge->debugDepth)
            maxDepth = std::max(maxDepth, id.edge->debugCC);
        id.drawn = true;
    }
    depth = std::max(-1, std::min(maxDepth, depth));
    if (depth == 0)  // draw all
        return;
	for (auto& id : ids) {
        if (IDType::edge != id.idType)
            continue;
		id.drawn = id.edge->debugDepth < depth && id.edge->debugCC >= depth;
	}
}

void DebuggerState::setIDTypes() {
    ids.clear();
    auto pushEdge = [this](const OpEdge* edge) {
        ids.emplace_back(edge);
        for (const auto& distance : edge->ray.distances) {
            ids.emplace_back(&distance);
        }
        for (const auto& pal : edge->pals) {
            ids.emplace_back(&pal);
        }
    };
	if (context->fillerStorage) {
        int index = 0;
        while (OpEdge* edge = context->fillerStorage->debugIndex(index++)) {
            pushEdge(edge);
        }
	}
	if (context->ccStorage) {
        int index = 0;
        while (OpEdge* edge = context->ccStorage->debugIndex(index++)) {
            pushEdge(edge);
        }
	}
    for (OpContour* contour : context->contours) {
        ids.emplace_back(contour);
        for (const auto& seg : contour->segments) {
            ids.emplace_back(&seg);
			for (auto& edge : seg.edges) {
                pushEdge(&edge);
			}
            for (const auto& sect : seg.sects.i) {
                ids.emplace_back(sect);
                if (sect->coincidenceID)
                    ids.emplace_back(sect, IDType::coincident);
                if (sect->unsectID)
                    ids.emplace_back(sect, IDType::unsectable);
            }
        }

    }
    if (const OpTree* tree = context->debugTree) {
        ids.emplace_back(tree);
	    for (int index = 0; index < tree->totalUsed; ++index) {
		    const OpLimb& limb = context->nthLimb(index);
            ids.emplace_back(&limb);
        }
    }
}

bool DebuggerState::update() {
    if (--updateCount >= 0)
        return false;
    ++updateAttempts;
    OpContext* newContext = fromFile(opFileName);
    if (!newContext) {
        updateCount = updateDelay;
        updateDelay += updateDelay;
        return false;
    }
    delete context;
    context = newContext;
    debugGlobalContext = nullptr; // debugGlobalContext = context;   // !!! needed?
    setIDTypes();
    redraw();
    updateAttempts = 0;
    updateDelay = 1;
    updateCount = 0;
    return true;
}
