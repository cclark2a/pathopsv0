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
    leftArrow,   // unicode 2190
    upArrow,     //         2191
    rightArrow,  //         2192
    downArrow,   //         2193
};

extern const std::array<std::string, 5> KeyCodeUTF8;

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

enum class KeyAction {
    act,
    show
};

struct KeyResult {
    std::string s;
    DrawLevel l = DrawLevel::none;
};

struct DebuggerState {
    DebuggerState();
    DebuggerEvent addEvent(SDL_Keymod , SDL_WindowID );
    DrawLevel doWheelCommon(const DebuggerEvent& debuggerEvent, int delta);
    void draw();  // same window as before
    DrawLevel eventCommon(const DebuggerEvent& );
    std::string floatToStr(float );
    DebuggerWindow* focus(SDL_WindowID );
    KeyResult keyEvent(const DebuggerEvent& debuggerEvent, KeyAction action);
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
    int depth = 0;
    int verboseLevel = 0;
    int maxUpdateAttempts = 10;
    SDL_AppResult error = (SDL_AppResult) 0;
    bool showContours = false;
    bool showEdges = true;
    bool showHex = false;
    bool showIntersections = false;
    bool showOutput = false;
    bool showSegments = false;
    bool showHelp = false;
};

#endif
