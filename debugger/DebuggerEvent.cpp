// (c) 2025, Cary Clark cclark2@gmail.com
#include "DebuggerState.h"
#include "DebuggerWindow.h"
#include "OpCurveCurve.h"

const std::array<std::string, 5> KeyCodeUTF8 {  "",  // key code : none
        "\xE2\x86\x90 ", "\xE2\x86\x91 ", "\xE2\x86\x92 ", "\xE2\x86\x93 " }; // lurd arrows

int DebuggerState::count(IDType idType) const {
    if (!context)
        return -2;
    int result = 0;
    switch (idType) {
        case IDType::contour:
            if (!context->contourStorage)
                return -1;
            return context->contourStorage->debugCount();
        break;
        case IDType::segment:
            if (!context->contourStorage)
                return -1;
            {
                OpContourIterator contourIter(context);
                for (auto contour : contourIter) {
                    result += (int) contour->segments.size();
                }
            }
        break;
        case IDType::edge:
            if (!context->contourStorage)
                return -1;
            {
                OpContourIterator contourIter(context);
                for (auto contour : contourIter) {
                    for (auto segment :  contour->segments) {
                        result += (int) segment.edges.size();
                    }
                }
            }
        break;
        case IDType::intersection:
            if (!context->contourStorage)
                return -1;
            {
                OpContourIterator contourIter(context);
                for (auto contour : contourIter) {
                    for (auto segment :  contour->segments) {
                        result += (int) segment.sects.i.size();
                    }
                }
            }
        break;
        default:
            ;
    }
    return result;
}

KeyResult DebuggerState::keyEvent(const DebuggerEvent& debuggerEvent, KeyAction action) {
    bool picTop = &pictureWindow == lastFocus;
    bool textTop = &textWindow == lastFocus;
    bool compareTop = &compareWindow == lastFocus;
    bool isLower = std::islower(debuggerEvent.key);
    KeyResult result;
    auto flip = [action, &result](bool& bit, std::string postfix) {
        switch (action) {
            case KeyAction::act:
                bit ^= true;
                result.l = DrawLevel::update;
            break;
            case KeyAction::show:
                if ("update" == postfix)  // !!! add way to choose verbs (hardcode for now)
                    result.s += (bit ? "defeat " : "allow ") + postfix;
                else
                    result.s += (bit ? "hide " : "show ") + postfix;
            break;
            default:
                OP_ASSERT(0);
        }
    };
    auto bump = [action, &result, isLower](Bumper& bumper) {
        int delta = isLower ? 1 : -1;
        switch (action) {
            case KeyAction::act: 
                bumper.bumpUp(delta);
                result.l = DrawLevel::update;
            break;
            case KeyAction::show:
                result.s += bumper.nextLabel(delta);
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
                result.l = pictureWindow.pan(OpVector(+pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::upArrow:
            if (KeyAction::act == action) {
                if (lastFocus && WheelTarget::zoomAndKeyPan != lastFocus->wheelTarget)
                    result.l = lastFocus->doWheel(debuggerEvent, +1);
                else if (picTop)
                    result.l = pictureWindow.pan(OpVector(0, +pan_factor * scale));
            }
            break;
        case (uint8_t) KeyCode::rightArrow:
            if (KeyAction::act == action && lastFocus
                    && WheelTarget::zoomAndKeyPan == lastFocus->wheelTarget)
                result.l = pictureWindow.pan(OpVector(-pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::downArrow:
            if (KeyAction::act == action) {
                if (lastFocus && WheelTarget::zoomAndKeyPan != lastFocus->wheelTarget)
                    result.l = lastFocus->doWheel(debuggerEvent, -1);
                else if (picTop)
                    result.l = pictureWindow.pan(OpVector(0, -pan_factor * scale));
            } else if (lastFocus && WheelTarget::zoomAndKeyPan == lastFocus->wheelTarget) {
                result.s = KeyCodeUTF8[1] + KeyCodeUTF8[2] + KeyCodeUTF8[3] + KeyCodeUTF8[4]
                        + "  pan picture left/up/right/down";
            } else if (lastFocus && WheelTarget::none != lastFocus->wheelTarget) {
                std::string verbage = WheelTargetVerbage[(int) lastFocus->wheelTarget];
                result.s = KeyCodeUTF8[(int) KeyCode::upArrow] + KeyCodeUTF8[debuggerEvent.key]
                        + "  " + verbage + " up/down";
                if (WheelTarget::zoomAndKeyZoom == lastFocus->wheelTarget)
                    result.s += " (" + STR(pictureWindow.zoomer) + ")";
                else if (WheelTarget::threshold == lastFocus->wheelTarget)
                    result.s += " (" + STR(pictureWindow.thresholdWheel) + " / "
                            + STR(pictureWindow.thresholdMultiplier) + ")";
                else if (WheelTarget::font == lastFocus->wheelTarget)
                    result.s += " (" + STR(lastFocus->fontSize) + ")";
            }
            break;
        case 'a': if (textTop) flip(textWindow.showAll, "all"); break;
        case 'b':
            if (!bitsToShow) {
                if (KeyAction::show == action)
                    result.s += "no bits to show";
            } else {
                flip(showBits, "bits");
                if (KeyAction::act == action)
                    showBits ? SDL_ShowWindow(compareWindow.window)        
                            : SDL_HideWindow(compareWindow.window);
            }
        break;
        case 'B': if (picTop) flip(pictureWindow.drawBounds, "bounds"); break;
        case 'C': flip(showContours, "contours (" + STR(count(IDType::contour)) + ")"); break;
        case 'c': 
            if (picTop) flip(pictureWindow.drawCenters, "center points"); 
            if (textTop) flip(textWindow.showCurveCurve, "curve/curve intersection");
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
        case 'e': flip(hideEdges, "edges (" + STR(count(IDType::edge)) + ")"); break;
        case 'E': if (textTop) flip(textWindow.showErasures, "tree erasures"); break;
        case 'f': 
            if (picTop) flip(pictureWindow.drawFill, "fill"); 
            if (textTop) flip(textWindow.showFull, "full");
        break;
        case 'g':
        case 'G': 
            if (picTop) bump(pictureWindow.gridLabel); 
        break;
        case 'h': if (picTop) flip(pictureWindow.drawHulls, "hulls"); break;
        case 'H': 
            if (picTop) flip(pictureWindow.drawEdgeHulls, "edge hull intersection points"); 
            if (textTop) flip(textWindow.showEdgeHulls, "edge hulls");
        break;
        case 'i': if (picTop) flip(pictureWindow.drawIDs, "IDs"); break;
        case 'I': flip(showIntersections, "intersections (" + STR(count(IDType::intersection)) + ")"); break;
        case 'j': if (textTop) flip(textWindow.showJoin, "join"); break;
        case 'k': if (picTop) flip(pictureWindow.drawControls, "controls"); break;
        case 'l': 
        case 'L':
            if (textTop && isLower) flip(textWindow.showLinks, "links"); 
            if (compareTop) bump(compareWindow.leftLabel); 
        break;
        case 'o': 
            if (compareTop)
                flip(compareWindow.overlay, "overlay");
            else
                flip(showOutput, "output");  // !!! unimplemented
            break;
        case 'p': 
            if (picTop) flip(pictureWindow.drawPoints, "points"); 
            if (textTop) flip(textWindow.showPoints, "points");
        break;
        case 'P': 
            if (KeyAction::act == action)
                playback();
            else
                result.s += "playback"; 
        break;
        case 'q': 
            flip(showDumps, "dumps");
            if (KeyAction::act == action)
                showDumps ? SDL_ShowWindow(dumpWindow.window)        
                        : SDL_HideWindow(dumpWindow.window);
        break;
        case 'r':
        case 'R': 
            if (compareTop) {
                bump(compareWindow.rightLabel);
                break;
            }
            if (isLower) {
                if (textTop)
                    flip(showRays, "rays");
                break;
            }
            if (KeyAction::act == action)
                record();
            else
                result.s += "record"; 
        break;
        case 's': flip(showSegments, "segments (" + STR(count(IDType::segment)) + ")"); break;
        case 't': 
            if (picTop) flip(pictureWindow.drawTangents, "tangents");
            if (textTop) flip(textWindow.showTree, "tree");
        break;
        case 'T': 
            if (picTop) flip(pictureWindow.drawTs, "t values"); 
            if (textTop) flip(textWindow.showTest, "test");
        break;
        case 'u':  // manual override; lockout updates from dump files
            flip(allowUpdate, "update");
        break;
        case 'v': if (picTop) flip(pictureWindow.drawValues, "point values"); break;
        case 'w': if (picTop) flip(pictureWindow.drawWindings, "windings"); break;
        case 'W': 
            if (KeyAction::act == action)
                        DebuggerState::RaiseWindows(); 
            else
                result.s += "bring windows to front";
            break;
        case 'x': 
            flip(showHex, "hex"); 
            if (KeyAction::act == action)  // !!! note code is duplicated in playback
                    defaultBase = showHex ? DebugBase::hex : DebugBase::dec;
            break;
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
        case '[':
            if (!currentDump) 
                break;
            if (KeyAction::act == action) {
                --currentDump;
                result.l = DrawLevel::file;
            } else
                result.s += "show dump " + STR(currentDump - 1);
            break;
        case ']':
            if (currentDump + 1 >= dumps.size())
                break;
            if (KeyAction::act == action) {
                ++currentDump;
                result.l = DrawLevel::file;
            } else
                result.s += "show dump " + STR(currentDump + 1);
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