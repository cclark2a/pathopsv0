// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DebuggerState_DEFINED
#define DebuggerState_DEFINED

#include "PictureWindow.h"
#include "TextWindow.h"
#include "HelpWindow.h"
#include <SDL3/SDL_init.h>

typedef uint32_t SDL_WindowID;
typedef uint16_t SDL_Keymod;

enum class KeyMods {
    none = 0,
    shift = 1,
    ctrl = 2,
    alt = 4
};

inline KeyMods operator|(KeyMods a, KeyMods b) {
	return (KeyMods) ((int) a | (int) b); }

inline KeyMods operator|=(KeyMods& a, KeyMods b) {
	return a = a | b; }

inline KeyMods operator&(KeyMods a, KeyMods b) {
	return (KeyMods) ((int) a & (int) b); }

inline KeyMods operator&=(KeyMods a, KeyMods b) {
	return a = a & b; }

enum class MouseAction {
    none,
    click,
    drag,
    move,
    wheel
};

// Use ascii characters as themselves. Map special keys to unused control characters
enum class KeyCode : uint8_t {
    none,
    downArrow,
    leftArrow,
    rightArrow,
    upArrow
};

struct DebuggerEvent {
    DebuggerEvent(DebuggerState* , SDL_Keymod , SDL_WindowID );
    DrawLevel doEvent();
    static int KeyModMultiplier(KeyMods );

    OpPoint mouse;      // current position
    OpPoint mouseDown;  // position when mouse button pressed
    OpPoint mouseLast;  // position sent to prior event (for finding drag delta)
    DebuggerWindow* focused = nullptr;
    MouseAction mouseAction = MouseAction::none;
    KeyMods keyMods = KeyMods::none;
    uint8_t key = 0;
    int wheel = 0;
};

struct DebuggerState {
    DebuggerState();
    DebuggerEvent addEvent(SDL_Keymod , SDL_WindowID );
    void draw();  // same window as before
    DrawLevel eventCommon(const DebuggerEvent& );
    std::string floatToStr(float );
    DebuggerWindow* focus(SDL_WindowID );
    void playback();
    void record();
    void redraw();  // changed window size, lay out again
    void setDepth(int );
    void setIDTypes();
    bool update();  // changed dump file, context data

    std::vector<OpType> ids;
    std::string opFileName;
    OpContext* context = nullptr;
    DebuggerWindow* lastFocus = nullptr;  // either picture window or text window
    time_t lastTime = 0;
    int updateAttempts = 0;
    int updateDelay = 1;
    int updateCount = 0;
    PictureWindow pictureWindow;
    TextWindow textWindow;
    HelpWindow helpWindow;
    OpVector threshold;
    float thresholdMultiplier = 1;
    int thresholdWheel = 0;
    int depth = 0;
    int verboseLevel = 0;
    int maxUpdateAttempts = 10;

    SDL_AppResult error = (SDL_AppResult) 0;
    bool showContours = false;
    bool showEdges = true;
    bool showEpsilon = true;
    bool showHex = false;
    bool showIntersections = false;
    bool showOutput = false;
    bool showSegments = false;
    bool tuneThreshold = false;
    bool showHelp = false;
    bool keyboardZoom = false;
    bool adjustFont = false;
};

#endif
