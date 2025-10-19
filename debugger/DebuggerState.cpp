// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpCurveCurve.h"
#include "DebuggerState.h"
#include <SDL3/SDL_error.h>

DebuggerState::DebuggerState() 
    : pictureWindow(this)
    , textWindow(this)
    , helpWindow(this) {
    opFileName = "dmp.txt";
    if (SDL_APP_CONTINUE != (error = pictureWindow.addFont(14)))
        OpDebugOut("Couldn't add picture font: " + std::string(SDL_GetError()) + "\n");
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
    helpWindow.update();
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

DebuggerWindow* DebuggerState::focus(SDL_WindowID id) {
    if (helpWindow.windowID == id)
        return lastFocus;
    lastFocus = pictureWindow.windowID == id ? (DebuggerWindow*) &pictureWindow :
            textWindow.windowID == id ? (DebuggerWindow*) &textWindow : nullptr;
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
    DEBUG_SET_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_SET_REQUIRED_VALUE(verboseLevel, maxUpdateAttempts);
    DEBUG_SET_REQUIRED_VALUE(maxUpdateAttempts, error);
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
    DEBUG_DUMP_REQUIRED_VALUE(depth, verboseLevel);
    DEBUG_DUMP_REQUIRED_VALUE(verboseLevel, maxUpdateAttempts);
    DEBUG_DUMP_REQUIRED_VALUE(maxUpdateAttempts, error);
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
    pictureWindow.update();
    textWindow.update();
    helpWindow.update();
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
