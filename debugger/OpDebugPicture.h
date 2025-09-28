// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebug.h"

#if OP_DEBUG_IMAGE

#include "PathOpsTypes.h"
#include "OpSegment.h"
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;
struct Window;
enum SDL_AppResult;

extern bool drawCentersOn;
extern bool drawControlsOn;
extern bool drawEdgesOn;
extern bool drawFillOn;
extern bool drawGridOn;
extern bool drawHexOn;
extern bool drawHullsOn;
extern bool drawIDsOn;
extern bool drawPointsOn;
extern bool drawSegmentsOn;
extern bool drawTangentsOn;
extern bool drawWindingsOn;
extern bool drawValuesOn;
extern bool drawGridLinear;
extern int debugPrecision;
extern int gridIntervals;

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

struct OpDebugPicture;

struct OpDebugAddPoly {
    void add(const PathOpsV0Lib::Curve& );
    void add(const OpEdge* );
    void add(const OpSegment* );
    void add(const OpContour* );

    OpDebugPicture* picture;
    const OpEdge* edge = nullptr;
    const OpSegment* segment = nullptr;
    const OpContour* contour = nullptr;
    bool continueCurve = false;  // true if contour extends loop
    bool addingFill = false;  // true if added is fill, false if added is frame
};

struct OpDebugPoly {
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
    void dump(OpDebugPicture& ) const;
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
//    OpDebugPoly* poly;
    OpPoint local;
    OpPoint device;
    float thickness = 1;
    DebugSprite sprite = DebugSprite::diamond;
};

struct OpDebugPicture {
    void addGrid();
    void addHulls();
    void addLabels();
    void addPoints();
    void addTangents();
    void addWindings();
    void colorPolys();

    void add(OpPoint , OpPoint , OpDebugAddPoly* );
    void add(std::vector<OpPoint>& points );
    void add(const OpCurve& , OpDebugAddPoly* );
    void addDevice(std::vector<OpPoint>& points, OpDebugPoly& );
    void addFittedBottom(std::string , float xPos, float right, uint32_t color);
    void addFittedSide(std::string , float yPos, float bottom, uint32_t color);
    void addLabel(std::string , OpPoint , uint32_t color);
    void addLine(OpPoint pt1, OpPoint pt2);
    void addTangent(OpDebugPoly& );
    OpDebugText& addText(std::string , OpPoint , uint32_t color, bool rotated = false);
    void addWinding(OpDebugPoly& );
    void append(OpPoint );
    void bootStrap();  // temporary to get things going
    void clear();
    std::string debugTextDump(size_t index);
    OpDebugPoly* findPoly(const OpEdge* );
    OpDebugPoly* findPoly(const OpSegment* );
    const NativeTextCache& getCache(size_t index);
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

    Window* window;
    OpDebugAddPoly addPoly;
    OpContext* context;
    std::vector<OpDebugPoly> polys;
    std::vector<OpDebugText> texts;
    std::vector<OpDebugPoint> points;
    std::vector<NativeTextCache> textCache;
    OpRect focus;  // local coordinates
    OpRect screen;  // device coordinates;
    OpVector wh; // screen w/h
    OpVector threshold;
    OpVector zoomOffset {0, 0};
    double scale; // factor to go from local to device
    float zoomFactor = 1;
    int depth = 0;
    int zoomer = 0;
    bool tuneThreshold = false;
};

struct Window {
    size_t addText(std::string str, uint32_t color);
    SDL_AppResult draw();
    void drawText();
    SDL_AppResult init(std::string name, OpVector offset);
    void pentrek_draw(char*, int width, int height, int pitch);

    OpDebugPicture debugPicture;
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* polysTexture = nullptr;
    int* buffer = nullptr;
    std::string name;
};

#endif
