// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebugPicture.h"
#include "OpCurveCurve.h"

DebuggerPoly& TextWindow::addRect(const OpRect& r, std::string s, uint32_t color) {
    uint32_t darker = color & 0xff3f3f3f;
    add(r, darker, 2);
    DebuggerPoly& result = add(r, color, 0);
    OpPoint pt = r.center();
    OpDebugText& text = addText(s, pt, black);
    const NativeTextCache& cache = getCache(text.cacheIndex);
    text.pt -= cache.size / 2;
    return result;
}

bool TextWindow::drawOne(DebuggerPoly& poly) {
    if (poly.edge) {
        if (poly.color == white)
            OpNop();
    }
    return true;
}

DrawLevel TextWindow::event(const DebuggerEvent& debuggerEvent) {
    if (MouseAction::move == debuggerEvent.mouseAction) {
        doEdge(DoType::hoverEdge, &debuggerEvent);
        return DrawLevel::draw;
    }
    return DrawLevel::none;
}

void TextWindow::redraw() {
    if (!context())
        return;
    OpCurveCurve* curveCurve = context()->debugCurveCurve;
    if (!curveCurve)
        return;
    clearWindow();
    addPoly.window = this;
    OpPoint localLocation(10, 10);
    std::string depthStr = "depth: " + STR(debuggerState->pictureWindow.depth) 
            + " / " + STR(curveCurve->depth);
    (void) addText(depthStr, localLocation, debugBlack, false);
    doEdge(DoType::addEdge, nullptr);
}

void TextWindow::doEdge(DoType doType, const DebuggerEvent* event) {
    static const int leftMargin = 10;
    OpPoint loc { leftMargin, 50 };
    OpVector wh { 50, 20 };
	for (const auto& edgeIter : edgeIterator) {
        const OpEdge& edge = *edgeIter;
        OpRect r(loc, loc + wh);
        if (r.right > screen.width() && r.left > leftMargin) {
            loc.x = leftMargin;
            loc.y += wh.dy + 8;
            r = OpRect(loc, loc + wh);
        }
        switch (doType) {
            case DoType::addEdge: {
                DebuggerPoly& edgeRect = addRect(r, STR(edge.id), lightGray);
                edgeRect.edge = &edge;
                } break;
            case DoType::hoverEdge: {
                DebuggerPoly* poly = findPoly(&edge);
                if (!poly)
                    break;
                bool mouseOverButton = r.contains(event->mouse);
                poly->color = mouseOverButton ? white : lightGray;
                poly = debuggerState->pictureWindow.findPoly(&edge);
                if (poly && poly->thickness)
                    poly->thickness = mouseOverButton ? 4 : 1;
                } break;
        }
        loc.x += wh.dx + 8;
    }
}
