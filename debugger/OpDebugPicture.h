// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebug.h"

#if OP_DEBUG_IMAGE

#include "PathOpsTypes.h"
struct DebuggerEvent;
struct DebuggerState;
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;
struct Window;
enum SDL_AppResult;
typedef uint32_t SDL_WindowID;
typedef uint16_t SDL_Keymod;

// all points are in device coordinates

// device bounds in float is rounded out
// extern std::string native_debugDump(size_t index);

struct NativeTextCache {
#if OP_DEBUG_DUMP
    std::string debugDump() const;
#endif

    std::string str;
    OpVector size;    // in device coordinates
    void* texture;
    uint32_t color;
};

// size_t native_addText(std::string str, uint32_t color);
// const NativeTextCache& native_cache(size_t index);

struct DebugSect {  // curve intersected with focus rectangle, and intersection pinned to rect
    OpPtT sect;
    bool pin;
};

struct DebuggerAddPoly {
    void add(const PathOpsV0Lib::Curve& );
    void add(const OpEdge* );
    void add(const OpSegment* );
    void add(const OpContour* );

    Window* window;
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    bool continueCurve = false;  // true if contour extends loop
    bool addingFill = false;  // true if added is fill, false if added is frame
};

struct DebuggerPoly {
#if OP_DEBUG_DUMP
    void dump() const;
#endif

    PathOpsV0Lib::Curve c;
    PathOpsV0Lib::CurveData cData;  // used by construction lines
    static constexpr float fill_thickness = 0;
    std::vector<OpPoint> local;    // lines used to draw, in local coordinates
    std::vector<OpPoint> device;    // lines used to draw, in device coordinates
    std::vector<size_t> contours;  // index for each device contour
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    float thickness = 1;    // special value for fill
    uint32_t color = debugBlack;
    float tStart = 0;
    float tEnd = 1;
    bool isPrimary = false;
};

struct OpDebugText {
#if OP_DEBUG_DUMP
    void dump(Window& ) const;
#endif
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    OpPoint pt;    // in device coordinates
    OpPoint debugLocal;
    size_t cacheIndex;
    bool vertical = false;
};

struct OpDebugPoint {
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
//    DebuggerPoly* poly;
    OpPoint local;
    OpPoint device;
    float thickness = 1;
    DebugSprite sprite = DebugSprite::diamond;
};

enum class DrawLevel {
    none,  // do nothing
    draw,  // draw the polys (e.g. in case color changed)
    update, // rebuild polys
    file,  // rebuild context
};

struct Window {
    Window();
    void add(const OpCurve& , DebuggerAddPoly* );
    DebuggerPoly& add(const OpRect& , uint32_t color, float thickness);
    void add(OpPoint , OpPoint , DebuggerAddPoly* );
    void add(std::vector<OpPoint>& points );
    void addLine(OpPoint pt1, OpPoint pt2);
    size_t addText(std::string str, uint32_t color);
    OpDebugText& addText(std::string , OpPoint , uint32_t color, bool rotated = false);
    SDL_AppResult allocateBuffers(int width, int height);
    void append(OpPoint );
    void clearWindow();
    OpContext* context();
    SDL_AppResult draw();
    virtual bool drawOne(DebuggerPoly& ) = 0;
    void drawText();
    virtual DrawLevel event(const DebuggerEvent& ) = 0;
    DebuggerPoly* findPoly(const OpEdge* );
    DebuggerPoly* findPoly(const OpSegment* );
    const NativeTextCache& getCache(size_t index);
    SDL_AppResult init(std::string name, OpVector offset);
    void pentrek_draw(char*, int width, int height, int pitch);
    void setSize();
#if OP_DEBUG_DUMP
    std::string debugTextDump(size_t index);
    void dumpWindow();
#endif
    DebuggerState* debuggerState;
    DebuggerAddPoly addPoly;
    std::vector<DebuggerPoly> polys;
    std::vector<OpDebugText> texts;
    std::vector<OpDebugPoint> points;
    std::vector<NativeTextCache> textCache;
    OpRect focus;  // local coordinates
    OpRect screen;  // device coordinates;
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* polysTexture = nullptr;
    int* buffer = nullptr;
    std::string name;
    OpVector threshold;
    OpVector windowSize;
    int windowID;
};

struct PictureWindow : public Window {
    void addGrid();
    void addHulls();
    void addLabels();
    void addPoints();
    void addTangents();
    void addWindings();
    void addDevice(std::vector<OpPoint>& points, DebuggerPoly& );
    void addFittedBottom(std::string , float xPos, float right, uint32_t color);
    void addFittedSide(std::string , float yPos, float bottom, uint32_t color);
    void addLabel(std::string , OpPoint , uint32_t color);
    void addTangent(DebuggerPoly& );
    void addWinding(DebuggerPoly& );
    void clear();
    void colorPolys();
    bool drawOne(DebuggerPoly& ) override;
    DrawLevel event(const DebuggerEvent& ) override;
    void move(OpVector v);  // v is in screen coordinates
    void pan(OpVector v);  // v is percentage of screen
    void redraw();
    void setDepth(int );
    void setDevice();
    OpPoint toLocal(OpPoint p);
    OpPoint toDevice(OpPoint p);
    bool touches(const OpRect& bounds);
    void zoom(int factor);
#if OP_DEBUG_DUMP
    void dump();
#endif

    OpVector zoomOffset {0, 0};
    double scale = 0; // factor to go from local to device (zero is uninitialized)
    float zoomFactor = 1;
    int depth = 0;
    int zoomer = 0;
    int debugPrecision = 0;
    int gridIntervals = 8;
    bool tuneThreshold = false;
    bool drawCentersOn = false;
    bool drawControlsOn = false;
    bool drawEdgesOn = true;
    bool drawFillOn = false;
    bool drawGridOn = true;
    bool drawHexOn = false;
    bool drawHullsOn = false;
    bool drawIDsOn = true;
    bool drawPointsOn = true;
    bool drawSegmentsOn = false;
    bool drawTangentsOn = false;
    bool drawWindingsOn = true;
    bool drawValuesOn = true;
    bool drawGridLinear = false;
};

enum class DoType {
    addEdge,
    hoverEdge,
};

struct TextWindow : public Window {
    DebuggerPoly& addRect(const OpRect& , std::string , uint32_t color);
    void doEdge(DoType , const DebuggerEvent* );
    bool drawOne(DebuggerPoly& ) override;
    DrawLevel event(const DebuggerEvent& );
    void redraw();

};

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

extern int keyModMultiplier(KeyMods mods);

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

    OpPoint mouse;      // current position
    OpPoint mouseDown;  // position when mouse button pressed
    OpPoint mouseLast;  // position sent to prior event (for finding drag delta)
    Window* focused;
    MouseAction mouseAction = MouseAction::none;
    KeyMods keyMods = KeyMods::none;
    uint8_t key = 0;
    int wheel = 0;
};

struct DebuggerState {
    DebuggerState();
    DebuggerEvent addEvent(SDL_Keymod , SDL_WindowID );
    void draw();
    Window* focus(SDL_WindowID );
    void redraw();
    void update();

    std::string opFileName;
    OpContext* context = nullptr;
    PictureWindow pictureWindow;
    TextWindow textWindow;
};


#endif
