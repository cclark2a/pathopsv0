// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image
#include "OpDebug.h"
#include "PathOpsTypes.h"
#include <time.h>

struct DebuggerEvent;
struct DebuggerState;
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;
struct TTF_Font;
struct Window;
enum SDL_AppResult;
typedef uint32_t SDL_WindowID;
typedef uint16_t SDL_Keymod;

static const int verboseLevel = 1;
static const int maxUpdateAttempts = 10;

// all points are in device coordinates

// device bounds in float is rounded out
// extern std::string native_debugDump(size_t index);

struct NativeTextCache {
#if OP_DEBUG_DUMP
    std::string debugDump() const;
#endif
    TTF_Font* font;
    std::string str;
    OpVector size;    // in device coordinates
    SDL_Texture* texture;
    uint32_t color;
};

// size_t native_addText(std::string str, uint32_t color);
// const NativeTextCache& native_cache(size_t index);

struct DebugSect {  // curve intersected with focus rectangle, and intersection pinned to rect
    OpPtT sect;
    bool pin;
};

enum class IDType {
    none,
    contour,
    segment,
    edge,
    intersection,
    coincident,
    unsectable,  // intersection
    unsectID,  // edge
    distance,
    pal,
    tree,
    limb,
};

struct OpType {
    OpType() { contour = nullptr; }
    OpType(const OpEdge* e);
    OpType(const OpSegment* s);
    OpType(const OpContour* c);
    OpType(const OpIntersection* i, IDType t = IDType::intersection);
    OpType(const struct Distance* d);
    OpType(const struct EdgePal* p);
    OpType(const struct OpTree* t);
    OpType(const struct OpLimb* l);

    OpRect bounds;
    union {
        const OpContour* contour;
        const OpSegment* segment;
        const OpEdge* edge;
        const OpIntersection* intersection;
        const Distance* distance;
        const EdgePal* pal;
        const OpTree* tree;
        const OpLimb* limb;
    };
    IDType idType = IDType::none;
    int id = 0;
    bool inCcStorage = false;
    bool selected = false;
    bool drawn = true;
};

struct DebuggerAddPoly {
    void add(const PathOpsV0Lib::Curve& );
    void add(const OpEdge* );
    void add(const OpSegment* );
    void add(const OpContour* );

    DebuggerState* debuggerState;
    Window* window;
    OpType opType;
    bool continueCurve = false;  // true if contour extends loop
    bool addingFill = false;  // true if added is fill, false if added is frame
    bool monotonic = false;
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
    OpType opType;
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
    OpType opType;
    OpPoint pt;    // in device coordinates
    OpPoint debugLocal;
    size_t cacheIndex;
    bool vertical = false;
};

struct OpDebugPoint {
    OpType opType;
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

inline DrawLevel operator|(DrawLevel a, DrawLevel b) {
	return (DrawLevel) std::max((int) a, (int) b); }

inline DrawLevel operator|=(DrawLevel& a, DrawLevel b) {
	return a = a | b; }

struct Window {
    Window(DebuggerState* state);
    void add(const OpCurve& , DebuggerAddPoly* );
    DebuggerPoly& add(const OpRect& , uint32_t color, float thickness);
    void add(OpPoint , OpPoint , DebuggerAddPoly* );
    void add(std::vector<OpPoint>& points );
    void addLine(OpPoint pt1, OpPoint pt2);
    SDL_AppResult addFont(float fontSize, TTF_Font** result = nullptr);
    size_t addText(std::string , uint32_t color, TTF_Font* f = nullptr);
    OpDebugText& addText(std::string , OpPoint , uint32_t color, TTF_Font* = nullptr, 
            bool rotated = false);
    SDL_AppResult allocateBuffers(int width, int height);
    void append(OpPoint );
    void clearWindow();
    OpContext* context();
    void deleteTextCache();
    SDL_AppResult draw();
    virtual bool drawOne(DebuggerPoly& ) { return true; }
    void drawText();
    virtual DrawLevel event(const DebuggerEvent& ) { return DrawLevel::none; }
    DebuggerPoly* findPoly(const OpEdge* );
    DebuggerPoly* findPoly(const OpSegment* );
    DebuggerPoly* findPolyByID(int );
    const NativeTextCache& getCache(size_t index);
    SDL_AppResult init(std::string name, OpVector offset);
    void pentrek_draw(char*, int width, int height, int pitch);
    virtual void playback(const char*&  str) { playbackCommon(str); }
    void playbackCommon(const char*& );
    virtual std::string record() { return recordCommon(); };
    std::string recordCommon();
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
    OpRect screen { 0, 0, 1000, 1000 };  // device coordinates;
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* polysTexture = nullptr;
    TTF_Font* font = nullptr;
    int* buffer = nullptr;
    std::string name;
    int windowID;
};

struct PictureWindow : public Window {
    PictureWindow(DebuggerState* state)
        : Window(state) {
    }
    void addGrid();
    void addHulls();
    void addLabel(std::string , OpPoint , uint32_t color);
    void addLabels();
    void addPointLabel(OpPoint , OpType& );
    void addPoints();
    void addTangents();
    void addTs();
    void addWindings();
    void addDevice(std::vector<OpPoint>& points, DebuggerPoly& );
    void addEdgeHulls();
    void addFittedBottom(std::string , float xPos, float right, uint32_t color);
    void addFittedSide(std::string , float yPos, float bottom, uint32_t color);
    void addTangent(DebuggerPoly& );
    void addWinding(DebuggerPoly& );
    void clear();
    void colorPolys();
    bool drawOne(DebuggerPoly& ) override;
    uint32_t edgeColor(const OpEdge& );
    DrawLevel event(const DebuggerEvent& ) override;
    void move(OpVector v);  // v is in screen coordinates
    void pan(OpVector v);  // v is percentage of screen
    void playback(const char*& str ) override;
    std::string record() override;
    void redraw();
    void resolvePoints();
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
    int zoomer = 0;
//    int debugPrecision = 0;
    int gridIntervals = 8;
    bool drawCentersOn = false;
    bool drawControlsOn = false;
    bool drawEdgeHullsOn = false;
    bool drawFillOn = false;
    bool drawGridOn = true;
    bool drawHullsOn = false;
    bool drawIDsOn = true;
    bool drawPointsOn = true;
    bool drawTangentsOn = false;
    bool drawTsOn = false;
    bool drawValuesOn = true;
    bool drawWindingsOn = true;
    bool drawGridLinear = false;
    bool keyboardZoom = false;
};

typedef DrawLevel (*EventAction)(const DebuggerEvent* , struct TextWindow* , OpType& );

struct TextWindow : public Window {
    TextWindow(DebuggerState* state)
        : Window(state) {
    }
    DebuggerPoly& addRect(const OpRect& , std::string , uint32_t color);
    DrawLevel doType(EventAction , const DebuggerEvent* );
    DrawLevel event(const DebuggerEvent& ) override;
    void playback(const char*& str ) override;
    std::string record() override;
    void redraw();
    
    TTF_Font* detailFont;
    bool showAll = false;
    bool showAliases = false;
    bool showCurveCurve = false;
    bool showFull = false;
    bool showEdgeHulls = false;
    bool showLinks = false;
    bool showPoints = false;
    bool showRays = false;
    bool showTree = false;
};

struct HelpWindow : public Window {
    HelpWindow(DebuggerState* state)
        : Window(state) {
        screen = { 0, 0, 365, 1000 };
    }
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    void redraw();  // draw help corresponding to topmost window
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
    Window* focused = nullptr;
    MouseAction mouseAction = MouseAction::none;
    KeyMods keyMods = KeyMods::none;
    uint8_t key = 0;
    int wheel = 0;
};

struct DebuggerState {
    DebuggerState();
    DebuggerEvent addEvent(SDL_Keymod , SDL_WindowID );
    void draw();
    DrawLevel eventCommon(const DebuggerEvent& );
    std::string floatToStr(float );
    Window* focus(SDL_WindowID );
    void playback();
    void record();
    void redraw();
    void setDepth(int );
    void setIDTypes();
    bool update();

    std::vector<OpType> ids;
    std::string opFileName;
    OpContext* context = nullptr;
    Window* lastFocus = nullptr;  // either picture window or text window
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

    SDL_AppResult error = (SDL_AppResult) 0;
    bool drawContoursOn = false;
    bool drawEdgesOn = true;
    bool drawEpsilonOn = true;
    bool drawHexOn = false;
    bool drawIntersectionsOn = false;
    bool drawSegmentsOn = false;
    bool tuneThreshold = false;
    bool drawHelp = false;
};
