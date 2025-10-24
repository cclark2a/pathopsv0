// (c) 2025, Cary Clark cclark2@gmail.com
#include "HelpWindow.h"
#include "DebuggerState.h"
#include "OpCurveCurve.h"

HelpWindow::HelpWindow(DebuggerState* state)
    : DebuggerWindow(state) {
    screen = { 0, 0, 365, 1000 };
    if (SDL_APP_CONTINUE != (state->error = init("help", { -200, -200 })))
        OpDebugOut("Couldn't initialize help window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(window);
}

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel HelpWindow::event(const DebuggerEvent& event) {
    OP_ASSERT(debuggerState->lastFocus);
    return debuggerState->lastFocus->event(event);
}

// updated when picture and text window change focus
void HelpWindow::update() {
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
        showHide("c", picWin.drawCenters, "center points");
        showHide("C", debuggerState->showContours, "contours");
        nextLine("d / D  curve/curve intersection depth" + depthInfo);
        showHide("e", debuggerState->showEdges, "edges");
        showHide("E", debuggerState->showEpsilon, "epsilon");
        showHide("f", picWin.drawFill, "fill");
        nextLine("g  show grid");
        showHide("h", picWin.drawHulls, "hulls");
        showHide("H", picWin.drawEdgeHulls, "edge hull intersection points");
        showHide("i", picWin.drawIDs, "show IDs");
        showHide("I", debuggerState->showIntersections, "intersections");
        showHide("k", picWin.drawControls, "controls");
        showHide("p", picWin.drawPoints, "points");
        nextLine("P  playback");
        nextLine("R  record");
        showHide("s", debuggerState->showSegments, "segments");
        showHide("t", picWin.drawTangents, "tangents");
        showHide("T", picWin.drawTs, "t values");
        showHide("v", picWin.drawValues, "values");
        showHide("w", picWin.drawWindings, "windings");
        showHide("x", debuggerState->showHex, "hex");
        nextLine("0-9  precision: " + STR(debugPrecision));
        nextLine("~  tune threshold multiplier: " + thres);
        nextLine("?  hide help");
    } else {
        OP_ASSERT(&textWin == debuggerState->lastFocus);
        showHide("a", textWin.showAll, "all");
        showHide("A", textWin.showAliases, "aliases");
        showHide("C", debuggerState->showContours, "contours");
        nextLine("d / D  curve/curve intersection depth" + depthInfo);
        showHide("e", debuggerState->showEdges, "edges");
        showHide("f", textWin.showFull, "full relationship");
        showHide("h", textWin.showEdgeHulls, "edge hulls");
        showHide("I", debuggerState->showIntersections, "intersections");
        showHide("l", textWin.showLinks, "links");
        showHide("p", textWin.showPoints, "points");
        showHide("r", textWin.showRays, "rays");
        showHide("s", debuggerState->showSegments, "segments");
        showHide("t", textWin.showTree, "tree");
        showHide("x", debuggerState->showHex, "hex");
        nextLine("0-9  precision: " + STR(debugPrecision));
        nextLine("~  tune threshold multiplier: " + thres);
        nextLine("?  hide help");
    }
}

