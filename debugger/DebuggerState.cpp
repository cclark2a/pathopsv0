// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpCurveCurve.h"
#include "DebuggerState.h"
#include <SDL3/SDL_error.h>
#include <filesystem>

DebuggerState::DebuggerState() 
    : pictureWindow(this)
    , textWindow(this)
    , helpWindow(this) {
    opFileName = "dmp.txt";
}

void DebuggerState::draw() {
    pictureWindow.draw();
    textWindow.draw();
    helpWindow.update();
    helpWindow.draw();
}

DrawLevel DebuggerState::doWheelCommon(const DebuggerEvent& debuggerEvent, int delta) {
    DebuggerWindow* f = debuggerEvent.focused;
    if (!f || WheelTarget::font != f->wheelTarget)
        return DrawLevel::none;
    int fontSize = f->fontSize;
    int scale = DebuggerEvent::KeyModMultiplier(debuggerEvent.keyMods);
    fontSize += scale;
    fontSize = std::max(3, fontSize);
    if (f->fontSize == fontSize)
        return DrawLevel::none;
    f->fontSize = fontSize;
    if (SDL_APP_CONTINUE != (error = f->addFont(fontSize))) // deletes font cache
        OpDebugOut("Couldn't add text font: " + std::string(SDL_GetError()) + "\n");
    return DrawLevel::update;
}

DrawLevel DebuggerState::eventCommon(const DebuggerEvent& debuggerEvent) {
    if (debuggerEvent.wheel)
        return doWheelCommon(debuggerEvent, debuggerEvent.wheel);
    if (debuggerEvent.key)
        return keyEvent(debuggerEvent, KeyAction::act).l;
    return DrawLevel::none;
}

std::string DebuggerState::floatToStr(float f) {
    return showHex ? OpDebugDumpHex(f) : STR(f);
}

DebuggerWindow* DebuggerState::focus(SDL_WindowID id) {
    if (helpWindow.windowID == id)
        return lastFocus;
    lastFocus = pictureWindow.windowID == id ? (DebuggerWindow*) &pictureWindow :
            textWindow.windowID == id ? (DebuggerWindow*) &textWindow : nullptr;
//    OP_ASSERT(lastFocus);  // if left running, id may not match any window
    return lastFocus;
}

void DebuggerState::playback() {
    std::string buffer = dmpFileToStr("DebuggerState.txt");
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
    DEBUG_SET_REQUIRED_VALUE(helpWindow, depth);
    DEBUG_SET_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_SET_REQUIRED_VALUE(verboseLevel, maxUpdateAttempts);
    DEBUG_SET_REQUIRED_VALUE(maxUpdateAttempts, error);
    DEBUG_SET_BOOL(error, showContours);
    DEBUG_SET_BOOL(showContours, showEdges);
    DEBUG_SET_BOOL(showEdges, showHex);
    DEBUG_SET_BOOL(showHex, showIntersections);
    DEBUG_SET_BOOL(showIntersections, showOutput);
    DEBUG_SET_BOOL(showOutput, showSegments);
    DEBUG_SET_BOOL(showSegments, showHelp);
    pictureWindow.playback(str);
    textWindow.playback(str);
    helpWindow.playback(str);
}

void DebuggerState::record() {
    std::string s;
    for (auto& id : ids) {
        if (id.selected)
            s += "id:" + STR(id.id) + " ";
    }
    if (!s.empty())
        s.back() = '\n';
    // !!! add any additional global state here
    DEBUG_DUMP_REQUIRED_VALUE(helpWindow, depth);
    DEBUG_DUMP_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_DUMP_REQUIRED_VALUE(verboseLevel, maxUpdateAttempts);
    DEBUG_DUMP_REQUIRED_VALUE(maxUpdateAttempts, error);
    DEBUG_DUMP_BOOL(error, showContours);
    DEBUG_DUMP_BOOL(showContours, showEdges);
    DEBUG_DUMP_BOOL(showEdges, showHex);
    DEBUG_DUMP_BOOL(showHex, showIntersections);
    DEBUG_DUMP_BOOL(showIntersections, showOutput);
    DEBUG_DUMP_BOOL(showOutput, showSegments);
    DEBUG_DUMP_BOOL(showSegments, showHelp);
    s += pictureWindow.record();
    s += textWindow.record(); 
    s += helpWindow.record();
    std::string fileName = "DebuggerState.txt";
    std::filesystem::path fullPath = std::filesystem::absolute(fileName);
	FILE* file = fopen(fileName.c_str(), "w");
    if (file)
        OpDebugOut( "recording: " + fullPath.string() + "\n");
    else {
        OpDebugOut( "invalid path: " + fullPath.string() + "\n");
        return;
    }
    fwrite(&s[0], 1, s.size(), file);
	fclose(file);
}

void DebuggerState::redraw() {
    if (!context) 
        return;
    pictureWindow.update();
    textWindow.update();
    helpWindow.update();
    draw();
}

// -1: draw none ; 0: draw all ; > 0 draw matching depth
void DebuggerState::setDepth(int ) {
    int maxDepth = 0;
	for (auto& id : ids) {
        if (IDType::edge != id.type)
            continue;
        if (id.edge->debugDepth)
            maxDepth = std::max(maxDepth, id.edge->debugCC);
        id.drawn = true;
    }
    depth = std::max(-1, std::min(maxDepth, depth));
    if (depth == 0)  // draw all
        return;
	for (auto& id : ids) {
        if (IDType::edge != id.type)
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
    std::sort(ids.begin(), ids.end(), [](const OpType& a, const OpType& b) {
            return a.id < b.id; });
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
    debugGlobalContext = context;   // needed for OpDebugFormat; major rework to remove dependency
    setIDTypes();
    redraw();
    updateAttempts = 0;
    updateDelay = 1;
    updateCount = 0;
    return true;
}
