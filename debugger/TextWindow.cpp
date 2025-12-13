// (c) 2025, Cary Clark cclark2@gmail.com

#include "DebuggerState.h"
#include "OpCurveCurve.h"
#include "OpJoiner.h"
#include <sys/stat.h>

// !!! hackery
#if __APPLE__
#define TEXT_DETAIL_FONT_SIZE 14
#elif _WIN32
#define TEXT_DETAIL_FONT_SIZE 24
#else
#define TEXT_DETAIL_FONT_SIZE 18
#endif 

TextWindow::TextWindow(DebuggerState* state)
        : DebuggerWindow(state, WheelTarget::scroll) {
    if (SDL_APP_CONTINUE != (state->error = init("text", { -100, -100 })))
        OpDebugOut("Couldn't initialize text window: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (state->error = addFont(fontSize)))
        OpDebugOut("Couldn't add text font: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (state->error = addFont(TEXT_DETAIL_FONT_SIZE, &detailFont)))
        OpDebugOut("Couldn't add text detail font: " + std::string(SDL_GetError()) + "\n");
    topClip = 60;  // !!! should be set to 30 x # of button rows
    lineHeight = fontSize;
}

DebuggerPoly& TextWindow::addIdBox(const OpRect& r, std::string s, uint32_t color) {
    uint32_t darker = color & 0xff3f3f3f;
    add(r, darker, 2);
    DebuggerPoly& result = add(r, color, 0);
    OpPoint pt = r.center();
    OpDebugText& text = addText(s, pt, black);
    const NativeTextCache& cache = getCache(text.cacheIndex);
    text.pt -= cache.size / 2;
    return result;
}

// start here;
// change this to generate the set of interesting ids into an array
// use user choices to order that id set, add breaking lines, etc

DrawLevel TextWindow::doType(TextAction eventAction, const DebuggerEvent* event) {
    if (!context())
        return DrawLevel::none;
    static const int leftMargin = 10;
    static const int topMargin = 10;
    OpPoint loc { leftMargin, topMargin };
    DrawLevel result = DrawLevel::none;
    std::vector<const OpContour*> shownContours;
	for (auto& id : debuggerState->ids) {
        if (IDType::intersection == id.type) {
            if (!debuggerState->showIntersections)
                continue;
        } else if (IDType::segment == id.type) {
            if (!debuggerState->showSegments)
                continue;
        } else if (IDType::contour == id.type) {
            if (!debuggerState->showContours)
                continue;
            if (shownContours.end() == std::find(shownContours.begin(), shownContours.end(), 
                    id.contour))
                shownContours.push_back(id.contour);
            else
                continue;
        } else if (IDType::edge != id.type)
            continue;
        id.bounds = OpRect(loc, loc + boxWH);
        if (id.bounds.right > screen.width() && id.bounds.left > leftMargin) {
            loc.x = leftMargin;
            loc.y += boxWH.dy + 8;
            topClip = loc.y + boxWH.dy + topMargin;
            id.bounds = OpRect(loc, loc + boxWH);
        }
        result |= (*eventAction)(event, this, id);
        loc.x += boxWH.dx + 8;
    }
    return result;
}

static DrawLevel AddType(const DebuggerEvent* , TextWindow* textWindow, OpType& opType) {
    DebuggerPoly& polyRect = textWindow->addIdBox(opType.bounds, STR(opType.id), 
        opType.selected ? yellow : lightGray);
    polyRect.opType = opType;
    return DrawLevel::draw;
}

static DrawLevel DragType(const DebuggerEvent* , TextWindow* textWindow, OpType& opType) {
    // !!! not implemented, yet
        // move boxes around?
    return DrawLevel::none;
}

static DrawLevel HoverType(const DebuggerEvent* event, TextWindow* textWindow, OpType& opType) {
    DebuggerPoly* poly = textWindow->findPolyByID(opType.id);
    if (!poly)
        return DrawLevel::none;
    bool mouseOverButton = opType.bounds.contains(event->mouse);
    poly->color = opType.selected ? yellow : mouseOverButton ? white : lightGray;
    poly = textWindow->debuggerState->pictureWindow.findPolyByID(opType.id);
    if (poly && poly->thickness)
        poly->thickness = mouseOverButton ? 4 : 1;
    return DrawLevel::draw;
}

static DrawLevel SelectType(const DebuggerEvent* event, TextWindow* , OpType& opType) {
    if (!opType.bounds.contains(event->mouse))
        return DrawLevel::none;
    opType.selected ^= true;
    return DrawLevel::update;
}

DrawLevel TextWindow::doWheel(const DebuggerEvent& debuggerEvent, int delta) {
    int scale = DebuggerEvent::KeyModMultiplier(debuggerEvent.keyMods);
    return scroll(delta * scale);
}

DrawLevel TextWindow::event(const DebuggerEvent& debuggerEvent) {    
    if (DrawLevel common = debuggerState->eventCommon(debuggerEvent); DrawLevel::none != common)
        return common;
    if (debuggerEvent.wheel)
        return doWheel(debuggerEvent, debuggerEvent.wheel);
    if (MouseAction::drag == debuggerEvent.mouseAction)
        return doType(&DragType, &debuggerEvent);
    if (MouseAction::move == debuggerEvent.mouseAction)
        return doType(&HoverType, &debuggerEvent);
    if (MouseAction::click == debuggerEvent.mouseAction)
        return doType(&SelectType, &debuggerEvent);
    return DrawLevel::none;
}

extern DebugBase defaultBase;

void TextWindow::innerUpdate(int& safetyCheck) {
    if (++safetyCheck > 2) {
        OpDebugOut(std::string(__func__) + ": unexpected recursion: " + STR(safetyCheck) + "\n");
        exit(1);
    }
    if (!context())
        return;
    clearWindow();
    OpPoint localLocation(10, 10);
    // find box size from current font; create temporary, then remove it from draw list
    OpDebugText& text = addText("9999", {0, 0}, black);
    const NativeTextCache& cache = getCache(text.cacheIndex);
    boxWH = cache.size;
    boxWH += OpVector(10, 10);
    texts.pop_back();
    doType(&AddType, nullptr);
    int lastDetailHeight = detailHeight;  // re-pin scroll if changed
    detailHeight = topClip;
    // set position based on last update and last scroll wheel
    // find window height available
    // 
    auto addWrapped = [this](std::string s) {
        s = stringFormat(s, 100);
        const NativeTextCache& cache = getCache(addClipped(s, 
                { 10, (float) (detailHeight - scrollPos) }, black, detailFont).cacheIndex);
        detailHeight += cache.size.dy;
    };
    std::vector<const OpSegment*> shownSegs;
    std::vector<const OpContour*> shownContours;
	for (auto& id : debuggerState->ids) {
        if (!id.selected && !showAll)
            continue;
        defaultBase = debuggerState->showHex ? DebugBase::hex : DebugBase::dec;
        bool shownEdge = false;
        bool shownIntersection = false;
        bool shownSegment = false;
        std::string s;
        const OpSegment* segment = nullptr;
        if (IDType::segment == id.type)
            segment = id.segment;
        else if (IDType::edge == id.type)
            segment = id.edge->segment;
        else if (IDType::intersection == id.type)
            segment = id.intersection->segment;
        if (segment && shownSegs.end() == std::find(shownSegs.begin(), shownSegs.end(), segment))
            shownSegs.push_back(segment);  // only show segment once, when edges/intersections selected
        else
            segment = nullptr;
        if (showFull && segment) {
            s = segment->debugDumpFull();
            shownSegment = true;
        }
        if (!shownSegment && segment && debuggerState->showSegments) {
            s = segment->debugDump(DebugLevel::normal, defaultBase);
            shownSegment = true;
        }
        if (showPoints) {
            if (!shownSegment && IDType::segment == id.type) {
                s = id.segment->debugDump(DebugLevel::brief, defaultBase);
                shownSegment = true;
            } 
            if (IDType::edge == id.type) {
                std::vector<EdgeFilter> showFields = { EF::id, EF::startT, EF::endT, EF::curve, 
                    EF::iStart, EF::iEnd, EF::winding, EF::sum, EF::whichEnd_impl };
                OpSaveEF saveEF(showFields);
                s = id.edge->debugDump(DebugLevel::normal, defaultBase);
                shownEdge = true;
            } 
            if (IDType::intersection == id.type) {
                s = id.intersection->debugDump(DebugLevel::normal, defaultBase);
                shownIntersection = true;
            }
        } 
        if (!shownSegment && IDType::segment == id.type)
            s = id.segment->debugDump(DebugLevel::normal, defaultBase);
        if (!shownIntersection && IDType::intersection == id.type)
            s = id.intersection->debugDump(DebugLevel::normal, defaultBase);
        if (!shownEdge && IDType::edge == id.type)
            s = id.edge->debugDump(DebugLevel::normal, defaultBase);
        if (IDType::contour == id.type && shownContours.end() == std::find(shownContours.begin(), 
                shownContours.end(), id.contour)) {
            shownContours.push_back(id.contour);
            s = id.contour->debugDump(DebugLevel::normal, defaultBase);
        }
        if (s.empty())
            continue;
        addWrapped(s);
    }
    if (OpCurveCurve* cc = debuggerState->context->debugCurveCurve; cc && showCurveCurve) {
        std::string s = cc->debugDump(DebugLevel::normal, defaultBase);
        addWrapped(s);
        if (debuggerState->depth) {
            s = cc->debugDumpDepth(debuggerState->depth);
            addWrapped(s);
        }
    }
    if (showAliases) {
        std::string s = debuggerState->context->aliases.debugDump(DebugLevel::normal, defaultBase);
        addWrapped(s);
    }
    if (showJoin) {
        std::string s = debugDmpJoin(debuggerState->context, DebugLevel::normal, defaultBase);
        addWrapped(s);
    }
    if (showLinks) {
        std::string s = debugDmpLinks(debuggerState->context, DebugLevel::normal, defaultBase);
        addWrapped(s);
    }
    if (showTest && !testIn.empty())
        addWrapped(testIn);
    if (showTest && !debuggerState->context->debugOutPath.empty())
        addWrapped(debuggerState->context->debugOutPath);
    if (lastDetailHeight != detailHeight) {
        (void) scroll(0);
        innerUpdate(safetyCheck);
    }
}

void TextWindow::update() {
    // retrieve test first if changed
    auto readTest = [this](std::string filename, std::string& testCode) {
        struct stat info;
        std::string filePath = dmpFileToPath(filename);
        if (stat(filePath.c_str(), &info) == -1)
            return;
        auto readText = [filename, &testCode]() {
            testCode = dmpFileToStr(filename);
            return !testCode.empty();
        };
        if (info.st_mtime == lastTime)
            return;
        if (readText())
            lastTime = info.st_mtime;
        else if (++updateAttempts > maxUpdateAttempts) {
            OpDebugOut("failed to update\n"); 
            OP_ASSERT(0);
            readText();  // for debugging
            return;
        }
    };
    readTest(TestInFile, testIn);
    int safetyCheck = 0;
    innerUpdate(safetyCheck);
}

void TextWindow::playback(const char*& str) {
    playbackCommon(str);
    DEBUG_SET_BOOL(updateAttempts, showAll);
    DEBUG_SET_BOOL(showAll, showAliases);
    DEBUG_SET_BOOL(showAliases, showCurveCurve);
    DEBUG_SET_BOOL(showCurveCurve, showFull);
    DEBUG_SET_BOOL(showFull, showEdgeHulls);
    DEBUG_SET_BOOL(showEdgeHulls, showJoin);
    DEBUG_SET_BOOL(showJoin, showLinks);
    DEBUG_SET_BOOL(showLinks, showPoints);
    DEBUG_SET_BOOL(showPoints, showRays);
    DEBUG_SET_BOOL(showRays, showTest); 
    DEBUG_SET_BOOL(showTest, showTree); 
}

std::string TextWindow::record() {
    std::string s;
    s += recordCommon();
    DEBUG_DUMP_BOOL(updateAttempts, showAll);
    DEBUG_DUMP_BOOL(showAll, showAliases);
    DEBUG_DUMP_BOOL(showAliases, showCurveCurve);
    DEBUG_DUMP_BOOL(showCurveCurve, showFull);
    DEBUG_DUMP_BOOL(showFull, showEdgeHulls);
    DEBUG_DUMP_BOOL(showEdgeHulls, showJoin);
    DEBUG_DUMP_BOOL(showJoin, showLinks);
    DEBUG_DUMP_BOOL(showLinks, showPoints);
    DEBUG_DUMP_BOOL(showPoints, showRays);
    DEBUG_DUMP_BOOL(showRays, showTest);
    DEBUG_DUMP_BOOL(showTest, showTree);
    return s;
}

int TextWindow::canScroll() const {
    int detailArea = (int) screen.height() - topClip;
    return std::max(0, detailHeight - detailArea);
}

DrawLevel TextWindow::scroll(int wheel) {
    int lastPos = scrollPos;
    scrollPos += wheel * lineHeight;
    int scrollable = canScroll();
    scrollPos = std::max(0, std::min(scrollable, scrollPos));
    if (!wheel || lastPos == scrollPos)
        return DrawLevel::none;
    OpDebugOut("scrollPos: " + STR(scrollPos) + "\n");
    update();
    return DrawLevel::update;
}
