// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DebuggerState_DEFINED
#define DebuggerState_DEFINED

#include "CompareWindow.h"
#include "DumpWindow.h"
#include "HelpWindow.h"
#include "PictureWindow.h"
#include "TextWindow.h"
#include <SDL3/SDL_init.h>

typedef uint32_t SDL_WindowID;
typedef uint16_t SDL_Keymod;

extern SDL_AppResult Fail(std::string s);
extern SDL_AppResult Continue(std::string s);
extern void ReportError(std::string s);

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

struct DebuggerDump {
    bool update(struct DebuggerState* );
    std::string filename;
    OpContext* context = nullptr;
    time_t lastTime = 0;
    int maxUpdateAttempts = 16;
    int updateAttempts = 0;
    int updateDelay = 1;
    int updateCount = 0;
};

struct DebuggerState {
    DebuggerState();
    DebuggerEvent addEvent(SDL_Keymod , SDL_WindowID );
    SDL_AppResult checkForNewFiles();
    int count(IDType ) const;
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
    void update();
    void validate();

    std::vector<DebuggerDump> dumps;
    std::vector<OpType> ids;
    OpContext* context = nullptr;
    DebuggerWindow* lastFocus = nullptr;  // never help window
    PictureWindow pictureWindow;
    TextWindow textWindow;
    HelpWindow helpWindow;
    CompareWindow compareWindow;
    DumpWindow dumpWindow;
    size_t currentDump = 0;
    int depth = 0;
    int verboseLevel = 1;
    SDL_AppResult error = (SDL_AppResult) 0;
    bool showContours = false;
    bool showEdges = true;
    bool showHex = false;
    bool showIntersections = false;
    bool showOutput = false;
    bool showSegments = false;
    bool showHelp = false;
    bool showBits = false;
    bool showDumps = false;
    bool validation = true;  // turn on as needed
};

#endif
