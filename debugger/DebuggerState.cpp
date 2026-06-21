// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpCurveCurve.h"
#include "OpDebugRaster.h"
#include "DebuggerState.h"
#include <SDL3/SDL_error.h>
#include <filesystem>
#include <sys/stat.h>

// !!! hackery for now (include a dummy copy of PathOpsV0Lib::dumpSkiaOutPath())
// probably need to eliminate OpDebugTags from needing this
namespace PathOpsV0Lib {
    void dumpSkiaOutPath(Context*) {
    }
}

void ReportError(std::string s) {
    const char* sdlError = SDL_GetError();
    if (sdlError[0])
        s += ": " + std::string(sdlError);
    OpDebugOut(s + "\n"); 
 }

SDL_AppResult Continue(std::string s) {
    ReportError(s);
    return SDL_APP_CONTINUE; 
}

SDL_AppResult Fail(std::string s) {
    ReportError(s);
    return SDL_APP_FAILURE; 
}

bool DebuggerDump::update(DebuggerState* state) {
    if (--updateCount >= 0)
        return false;
    ++updateAttempts;
    OpContext* newContext = fromFile(filename);
    if (!newContext) {
        updateCount = updateDelay;
        updateDelay += updateDelay;
        return false;
    }
    delete context;
    context = newContext;
    debugGlobalContext = context;   // needed for OpDebugFormat; major rework to remove dependency
    state->update();
    updateAttempts = 0;
    updateDelay = 1;
    updateCount = 0;
    return true;
}

DebuggerState::DebuggerState() 
    : pictureWindow(this)
    , textWindow(this)
    , helpWindow(this)
    , compareWindow(this)
    , dumpWindow(this) {
}

SDL_AppResult DebuggerState::checkForNewFiles() {
    if (!allowUpdate)
        return SDL_APP_CONTINUE;
    struct stat info;
    size_t fileNumber = 0;
    int MAX_FILE_NUMBER = INT_MAX;  // !!! set when debugging debugger
    while (fileNumber < MAX_FILE_NUMBER) {
        std::string filename = DumpFile + STR(++fileNumber) + ".txt";
        std::string filePath = dmpFileToPath(filename);
        if (stat(filePath.c_str(), &info) == -1) {
            if (dumps.size() >= fileNumber)
                dumps.resize(fileNumber - 1);
            currentDump = std::max(0, std::min(clickDump, (int) dumps.size() - 1));
            break;
        }
        if (dumps.size() < fileNumber)
            dumps.emplace_back();
        OP_ASSERT(fileNumber <= dumps.size());
        DebuggerDump& dump = dumps[fileNumber - 1];
        dump.filename = filename;
        if (info.st_mtime == dump.lastTime)
            continue;
        if (dump.update(this)) {
            dump.lastTime = info.st_mtime;
            return SDL_APP_CONTINUE;
        } 
        if (dump.updateAttempts > dump.maxUpdateAttempts) {
            OP_ASSERT(0);
            dump.update(this);  // for debugging
            return Fail("failed to update: " + filename);
        }
    }
//    if (1 == fileNumber)  // all dumps have been deleted, but that should be OK
//        return Fail("no dmp found");
#if 0
    std::string bitsFilename = dmpFileToPath(BitsFile);
    bitsToShow = stat(bitsFilename.c_str(), &info) != -1;
    if (!bitsToShow) {
        if (showBits) {
            showBits = false;
            SDL_HideWindow(compareWindow.window);
        }
    } else if (info.st_mtime != compareWindow.lastTime) {
        compareWindow.update();
        compareWindow.lastTime = info.st_mtime;
    }
#endif
    return SDL_APP_CONTINUE;
}

void DebuggerState::draw() {
    pictureWindow.draw();
    textWindow.draw();
    helpWindow.update();
    helpWindow.draw();
    compareWindow.draw();
    dumpWindow.draw();
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
    if (SDL_APP_CONTINUE != (error = f->addFont((float) fontSize))) // deletes font cache
        ReportError("Couldn't add text font");
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
    lastFocus = pictureWindow.windowID == id ? (DebuggerWindow*) &pictureWindow
            : textWindow.windowID == id ? (DebuggerWindow*) &textWindow
            : compareWindow.windowID == id ? (DebuggerWindow*) &compareWindow 
            : dumpWindow.windowID == id ? (DebuggerWindow*) &dumpWindow 
            : nullptr;
//    OP_ASSERT(lastFocus);  // if left running, id may not match any window
    return lastFocus;
}

const std::string StateFile = "DebuggerState.txt";

void DebuggerState::playback() {
    std::string buffer = dmpFileToStr(StateFile);
    if (buffer.empty())
        return;
    const char* str = buffer.c_str();
    while (OpDebugOptional(str, "id")) {
        int id = (int) OpDebugReadSizeT(str);
        auto foundID = std::find_if(ids.begin(), ids.end(), [id](const OpType& opType) {
                return id == opType.id; });
        if (ids.end() != foundID)
            foundID->selected = true;
    }
    // !!! add any additional global state here
    DEBUG_SET_REQUIRED_VALUE(dumpWindow, clickDump);
    DEBUG_SET_REQUIRED_VALUE(currentDump, depth);
    DEBUG_SET_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_SET_REQUIRED_VALUE(verboseLevel, error);
    ASSERT_ORDERED(error, bitsToShow);  // don't restore
    DEBUG_SET_BOOL(bitsToShow, allowUpdate);
    DEBUG_SET_BOOL(allowUpdate, showContours);
    DEBUG_SET_BOOL(showContours, showEdges);
    DEBUG_SET_BOOL(showEdges, showHex);
    // !!! need some way to call a custom set function ?
    defaultBase = showHex ? DebugBase::hex : DebugBase::dec;
    DEBUG_SET_BOOL(showHex, showIntersections);
    DEBUG_SET_BOOL(showIntersections, showOutput);
    DEBUG_SET_BOOL(showOutput, showRays);
    DEBUG_SET_BOOL(showRays, showSegments);
    DEBUG_SET_BOOL(showSegments, showHelp);
    DEBUG_SET_BOOL(showHelp, showBits);
    DEBUG_SET_BOOL(showBits, showDumps);
    pictureWindow.playback(str);
    textWindow.playback(str);
    helpWindow.playback(str);
    compareWindow.playback(str);
    dumpWindow.playback(str);
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
    DEBUG_DUMP_REQUIRED_VALUE(dumpWindow, clickDump);
    DEBUG_DUMP_REQUIRED_VALUE(currentDump, depth);
    DEBUG_DUMP_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_DUMP_REQUIRED_VALUE(verboseLevel, error);
    ASSERT_ORDERED(error, bitsToShow);  // don't save
    DEBUG_DUMP_BOOL(bitsToShow, allowUpdate);
    DEBUG_DUMP_BOOL(allowUpdate, showContours);
    DEBUG_DUMP_BOOL(showContours, showEdges);
    DEBUG_DUMP_BOOL(showEdges, showHex);
    DEBUG_DUMP_BOOL(showHex, showIntersections);
    DEBUG_DUMP_BOOL(showIntersections, showOutput);
    DEBUG_DUMP_BOOL(showOutput, showRays);
    DEBUG_DUMP_BOOL(showRays, showSegments);
    DEBUG_DUMP_BOOL(showSegments, showHelp);
    DEBUG_DUMP_BOOL(showHelp, showBits);
    DEBUG_DUMP_BOOL(showBits, showDumps);
    s += pictureWindow.record();
    s += textWindow.record(); 
    s += helpWindow.record();
    s += compareWindow.record();
    s += dumpWindow.record();
    std::string fileName = dmpFileToPath(StateFile);
	FILE* file = fopen(fileName.c_str(), "w");
    if (file)
        OpDebugOut( "recording: " + fileName + "\n");
    else {
        OpDebugOut( "invalid path: " + fileName + "\n");
        return;
    }
    fwrite(&s[0], 1, s.size(), file);
	fclose(file);
}

void DebuggerState::redraw() {
    if (!context) 
        return;
#if OP_DEBUG
    if (validation)
        validate();
#endif
    pictureWindow.update();
    textWindow.update();
    helpWindow.update();
    compareWindow.update();
    dumpWindow.update();
    draw();
}

void DebuggerState::saveSelection() {
    selectedIDs.clear();
    for (const OpType& opType : ids) {
        if (opType.selected)
            selectedIDs.push_back(opType.id);
    }
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
    OpContourIterator contourIter(context);
    for (auto contour : contourIter) {
        for (int index = 0; index < contour->debugCurveData.size(); ++index)
            ids.emplace_back(contour, index);
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
    for (int selectedId : selectedIDs) {
        // !!! since ids is sorted, could use lower_bound or whatever...
        //     .. but if performance is really a problem, walk both (assuming selectedID is sorted)
        //        at the same time -- would be faster than existing or lower_bound
        auto idIter = std::find_if(ids.begin(), ids.end(), [selectedId](const OpType& test) {
                return test.id == selectedId ; });
        if (idIter != ids.end())
            (*idIter).selected = true;
    }
    selectedIDs.clear();
}

void DebuggerState::update() {
    OP_ASSERT(currentDump < dumps.size());
    context = dumps[currentDump].context;
    DebugRaster* debugRaster = context->debugRaster;
    bitsToShow = debugRaster && !debugRaster->sampleSets.empty();
    if (!bitsToShow) {
        if (showBits) {
            showBits = false;
            SDL_HideWindow(compareWindow.window);
        }
    } else
        compareWindow.update();
    setIDTypes();
    redraw();
}

#if OP_DEBUG
void DebuggerState::validate() {
    for (OpType& id : ids) {
        id.validate();
    }
//    context->debugValidate();  // !!! don't want to assert while debugging bad context
    pictureWindow.validate();
    textWindow.validate();
    helpWindow.validate();
    compareWindow.validate();
    dumpWindow.validate();
}
#endif