// (c) 2025, Cary Clark cclark2@gmail.com
#include "DebuggerState.h"
#include "DebuggerWindow.h"
#include "OpCurveCurve.h"

enum class KeyAction {
    act,
    show
};

std::string KeyEvent(DebuggerState& state, const DebuggerEvent& debuggerEvent, KeyAction action) {
    PictureWindow& picWin = state.pictureWindow;
    TextWindow& textWin = state.textWindow;
    bool picTop = &picWin == state.lastFocus;
    bool textTop = &textWin == state.lastFocus;
    auto flip = [debuggerEvent, action](bool& bit, std::string postfix) {
        std::string result;
        switch (action) {
            case KeyAction::act:
                bit ^= true;
            break;
            case KeyAction::show:
                result = (bit ? "  hide " : "  show ") + postfix;
            break;
        }
        return result;
    };
    auto level = [debuggerEvent, action](bool& bit, int val, std::string postfix) {
        std::string result;
        switch (action) {
            case KeyAction::act:
                bit ^= true;
            break;
            case KeyAction::show:
                result = (bit ? " disable " : " enable ") + postfix;
                if (val)
                    result += " (" + STR(val) + ")";
            break;
        }
        return result;
    };
    auto bumpEnum = [action](int& enumIndex, const std::vector<std::string>& strs) {
        std::string result;
        switch (action) {
            case KeyAction::act:
                enumIndex += 1;
                if (enumIndex >= strs.size())
                    enumIndex = 0;
            break;
            case KeyAction::show:
                result = strs[enumIndex];
            break;
        }
        return result;
    };
    std::string result;
    if (KeyAction::show == action)
        result = std::string(1, debuggerEvent.key) + " "; 
    switch (debuggerEvent.key) {
        case 'a': if (textTop) result += flip(textWin.showAll, "all"); break;
        case 'A': if (textTop) result += flip(textWin.showAliases, "aliases"); break;
        case 'C': result += flip(state.showContours, "contours"); break;
        case 'c': if (picTop) result += flip(picWin.drawCenters, "center points"); break;
        case 'd': if (KeyAction::act == action) state.setDepth(++state.depth); break;
        case 'D': 
            if (KeyAction::act == action) 
                state.setDepth(--state.depth);
            else if (OpCurveCurve* curveCurve = state.context->debugCurveCurve)
                result = "d / D  curve/curve intersection depth: " 
                        + STR(state.depth) + " / " + STR(curveCurve->depth);
        break; 
        case 'e': result += flip(state.showEdges, "edges"); break;
        case 'E': result += flip(state.showEpsilon, "epsilon"); break;
        case 'f': if (picTop) result += flip(picWin.drawFill, "fill"); break;
        case 'F': {
            DebuggerWindow* f = debuggerEvent.focused;
            int fontSize = f ? f->fontSize : 0;
            result += level(state.adjustFont, fontSize, "font size"); 
        } break;
        case 'g': result += bumpEnum(*(int*)&picWin.drawGrid, drawGridStrs); break;
        case 'h': if (picTop) result += flip(picWin.drawHulls, "hulls"); break;
        case 'H': if (picTop) result += flip(picWin.drawEdgeHulls, 
                "edge hull intersection points"); break;
        case 'i': if (picTop) result += flip(picWin.drawIDs, "IDs"); break;
        case 'I': result += flip(state.showIntersections, "intersections"); break;
        case 'j': if (textTop) result += flip(textWin.showJoin, "join"); break;
        case 'k': if (picTop) result += flip(picWin.drawControls, "controls"); break;
        case 'l': if (textTop) result += flip(textWin.showLinks, "links"); break;
        case 'o': result += flip(state.showOutput, "output"); break;
        case 'p': 
            if (picTop) result += flip(picWin.drawPoints, "points"); 
            if (textTop) result += flip(textWin.showPoints, "points");
        break;
        case 'P': result += "playback"; break;
        case 'R': result += "record"; break;
        case 's': result += flip(state.showSegments, "segments"); break;
        case 't': 
            if (picTop) result += flip(picWin.drawTangents, "tangents");
            if (textTop) result += flip(textWin.showTree, "tree");
        break;
        case 'T': if (picTop) result += flip(picWin.drawTs, "t values"); break;
        case 'v': if (picTop) result += flip(picWin.drawValues, "point values"); break;
        case 'w': if (picTop) result += flip(picWin.drawWindings, "windings"); break;
        case 'x': result += flip(state.showHex, "hex"); break;
        case 'z': 
            if (KeyAction::act == action)
                  state.keyboardZoom ^= true;
            else
                result += state.keyboardZoom ? "arrows pan" : "arrows zoom";
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
            if (KeyAction::act == action)
                debugPrecision = debuggerEvent.key - '0';
            else if ('0' == debuggerEvent.key)
                result = "0-9  precision: " + STR(debugPrecision);
            break;
        case '-':
            if (KeyAction::act == action)
                debugPrecision = -1;
            else
                result += "show epsilon";
            break;
        case '~':
            if (KeyAction::act == action)
                state.tuneThreshold ^= true;
            else
                result += STR(state.thresholdWheel) + " / "
                        + STR(state.thresholdMultiplier);
            break;
        case '?':
            result += flip(state.showHelp, "help");
            if (KeyAction::act == action)
                state.showHelp ? SDL_ShowWindow(state.helpWindow.window)        
                        : SDL_HideWindow(state.helpWindow.window);
            break;
    }
    return result;
}