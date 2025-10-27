// (c) 2025, Cary Clark cclark2@gmail.com
#include "DebuggerState.h"
#include "DebuggerWindow.h"
#include "OpCurveCurve.h"

const std::array<std::string, 5> KeyCodeUTF8 {  "",  // key code : none
        "\xE2\x86\x90 ", "\xE2\x86\x91 ", "\xE2\x86\x92 ", "\xE2\x86\x93 " }; // lurd arrows

KeyResult DebuggerState::keyEvent(const DebuggerEvent& debuggerEvent, KeyAction action) {
    PictureWindow& picWin = pictureWindow;
    TextWindow& textWin = textWindow;
    bool picTop = &picWin == lastFocus;
    bool textTop = &textWin == lastFocus;
    KeyResult result;
    auto flip = [action, &result](bool& bit, std::string postfix) {
        switch (action) {
            case KeyAction::act:
                bit ^= true;
                result.l = DrawLevel::update;
            break;
            case KeyAction::show:
                result.s += (bit ? "hide " : "show ") + postfix;
            break;
            default:
                OP_ASSERT(0);
        }
    };
    auto bumpEnum = [action, &result](int& enumIndex, const std::vector<std::string>& strs) {
        switch (action) {
            case KeyAction::act:
                enumIndex += 1;
                if (enumIndex >= strs.size())
                    enumIndex = 0;
                result.l = DrawLevel::update;
            break;
            case KeyAction::show:
                result.s += strs[enumIndex];
            break;
            default:
                OP_ASSERT(0);
        }
    };
    if (KeyAction::show == action)
        result.s = std::string(1, debuggerEvent.key) + "  "; 
    int scale = DebuggerEvent::KeyModMultiplier(debuggerEvent.keyMods);
    constexpr float pan_factor = 1.f / 8;
    switch (debuggerEvent.key) {
        case (uint8_t) KeyCode::leftArrow:
            if (KeyAction::act == action && lastFocus
                    && WheelTarget::zoomAndKeyPan == lastFocus->wheelTarget)
                result.l = picWin.pan(OpVector(+pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::upArrow:
            if (KeyAction::act == action) {
                if (lastFocus && WheelTarget::zoomAndKeyPan != lastFocus->wheelTarget)
                    result.l = lastFocus->doWheel(debuggerEvent, +1);
                else if (picTop)
                    result.l = picWin.pan(OpVector(0, +pan_factor * scale));
            }
            break;
        case (uint8_t) KeyCode::rightArrow:
            if (KeyAction::act == action && lastFocus
                    && WheelTarget::zoomAndKeyPan == lastFocus->wheelTarget)
                result.l = picWin.pan(OpVector(-pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::downArrow:
            if (KeyAction::act == action) {
                if (lastFocus && WheelTarget::zoomAndKeyPan != lastFocus->wheelTarget)
                    result.l = lastFocus->doWheel(debuggerEvent, -1);
                else if (picTop)
                    result.l = picWin.pan(OpVector(0, -pan_factor * scale));
            } else if (lastFocus && WheelTarget::zoomAndKeyPan == lastFocus->wheelTarget) {
                result.s = KeyCodeUTF8[1] + KeyCodeUTF8[2] + KeyCodeUTF8[3] + KeyCodeUTF8[4]
                        + "  pan picture left/up/right/down";
            } else if (lastFocus && WheelTarget::none != lastFocus->wheelTarget) {
                std::string verbage = WheelTargetVerbage[(int) lastFocus->wheelTarget];
                result.s = KeyCodeUTF8[(int) KeyCode::upArrow] + KeyCodeUTF8[debuggerEvent.key]
                        + "  " + verbage + " up/down";
                if (WheelTarget::zoomAndKeyZoom == lastFocus->wheelTarget)
                    result.s += " (" + STR(picWin.zoomer) + ")";
                else if (WheelTarget::threshold == lastFocus->wheelTarget)
                    result.s += " (" + STR(picWin.thresholdWheel) + " / "
                            + STR(picWin.thresholdMultiplier) + ")";
                else if (WheelTarget::font == lastFocus->wheelTarget)
                    result.s += " (" + STR(lastFocus->fontSize) + ")";
            }
            break;
        case 'a': if (textTop) flip(textWin.showAll, "all"); break;
        case 'A': if (textTop) flip(textWin.showAliases, "aliases"); break;
        case 'C': flip(showContours, "contours"); break;
        case 'c': 
            if (picTop) flip(picWin.drawCenters, "center points"); 
            if (textTop) flip(textWin.showCurveCurve, "curve/curve intersection");
        break;
        case 'd': 
            if (KeyAction::act == action) {
                setDepth(++depth);
                result.l = DrawLevel::update;
            } 
        break;
        case 'D': 
            if (KeyAction::act == action) {
                setDepth(--depth);
                result.l = DrawLevel::update;
            } else if (OpCurveCurve* curveCurve = context->debugCurveCurve)
                result.s = "d / D  curve/curve intersection depth: " 
                        + STR(depth) + " / " + STR(curveCurve->depth);
        break; 
        case 'e': flip(showEdges, "edges"); break;
        case 'f': 
            if (picTop) flip(picWin.drawFill, "fill"); 
            if (textTop) flip(textWin.showFull, "full");
        break;
        case 'g': if (picTop) bumpEnum(*(int*)&picWin.drawGrid, drawGridStrs); break;
        case 'h': if (picTop) flip(picWin.drawHulls, "hulls"); break;
        case 'H': 
            if (picTop) flip(picWin.drawEdgeHulls, "edge hull intersection points"); 
            if (textTop) flip(textWin.showEdgeHulls, "edge hulls");
        break;
        case 'i': if (picTop) flip(picWin.drawIDs, "IDs"); break;
        case 'I': flip(showIntersections, "intersections"); break;
        case 'j': if (textTop) flip(textWin.showJoin, "join"); break;
        case 'k': if (picTop) flip(picWin.drawControls, "controls"); break;
        case 'l': if (textTop) flip(textWin.showLinks, "links"); break;
        case 'o': flip(showOutput, "output"); break;
        case 'p': 
            if (picTop) flip(picWin.drawPoints, "points"); 
            if (textTop) flip(textWin.showPoints, "points");
        break;
        case 'P': 
            if (KeyAction::act == action)
                playback();
            else
                result.s += "playback"; 
        break;
        case 'R': 
            if (KeyAction::act == action)
                record();
            else
                result.s += "record"; 
        break;
        case 's': flip(showSegments, "segments"); break;
        case 't': 
            if (picTop) flip(picWin.drawTangents, "tangents");
            if (textTop) flip(textWin.showTree, "tree");
        break;
        case 'T': if (picTop) flip(picWin.drawTs, "t values"); break;
        case 'v': if (picTop) flip(picWin.drawValues, "point values"); break;
        case 'w': if (picTop) flip(picWin.drawWindings, "windings"); break;
        case 'x': flip(showHex, "hex"); break;
        case 'z': 
            if (lastFocus) {
                WheelTarget next = (WheelTarget) ((int) lastFocus->wheelTarget + 1);
                if (picTop && next > WheelTarget::font) 
                    next = WheelTarget::zoomAndKeyPan;
                if (textTop && next > WheelTarget::scroll)
                    next = WheelTarget::font;
                if (KeyAction::act == action) {
                    lastFocus->wheelTarget = next;
                    result.l = DrawLevel::update;
                } else
                    result.s += "up/down arrows " + WheelTargetVerbage[(int) next];
            }
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
            if (KeyAction::act == action) {
                debugPrecision = debuggerEvent.key - '0';
                result.l = DrawLevel::update;
            } else if ('0' == debuggerEvent.key)
                result.s = "0-9  precision: " + STR(debugPrecision);
            break;
        case '-':
            if (KeyAction::act == action) {
                debugPrecision = -1;
                result.l = DrawLevel::update;
            } else
                result.s += "show epsilon";
            break;
         case '?':
            flip(showHelp, "help");
            if (KeyAction::act == action)
                showHelp ? SDL_ShowWindow(helpWindow.window)        
                        : SDL_HideWindow(helpWindow.window);
            break;
        default:
            // unassigned keys do nothing
            break;
    }
    if (KeyAction::show == action && result.s.size() <= 3)
        result.s.clear();
    return result;
}