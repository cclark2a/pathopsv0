// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DebuggerTypes_DEFINED
#define DebuggerTypes_DEFINED

#include "OpMath.h"
#include "OpDebugColor.h"
#include "PathOpsTypes.h"

struct DebugOutput;
struct DebuggerState;
struct DebuggerWindow;
struct SDL_Texture;
struct TTF_Font;

extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

// !!! hackery
#if __APPLE__
#define TEXT_FONT_SIZE 14
#define TEXT_DETAIL_FONT_SIZE 14
#elif _WIN32
#define TEXT_FONT_SIZE 24
#define TEXT_DETAIL_FONT_SIZE 24
#else
#define TEXT_FONT_SIZE 18
#define TEXT_DETAIL_FONT_SIZE 18
#endif 

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

struct NativeTextCache {
#if OP_DEBUG_DUMP
    std::string debugDump() const;
#endif
    TTF_Font* font;
    DebuggerWindow* window;
    std::string str;
    OpVector size;    // in device coordinates
    SDL_Texture* texture;
    uint32_t color;
};

struct OpType {
    OpType() { contour = nullptr; }
    OpType(const OpEdge* e);
    OpType(const OpSegment* s);
    OpType(const OpContour* c, int cIndex);
    OpType(const OpIntersection* i, IDType t = IDType::intersection);
    OpType(const struct Distance* d);
    OpType(const struct EdgePal* p);
    OpType(const struct OpTree* t);
    OpType(const struct OpLimb* l);
    void validate() const;

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
    int id = 0;
    int curveIndex = -1;  // only used by contour
    IDType type = IDType::none;
    bool inCcStorage = false;
    bool selected = false;
    bool drawn = true;
};

struct OpDebugPoint {
    OpType opType;
//    DebuggerPoly* poly;
    OpPoint local;
    OpPoint device;
    float thickness = 1;
    DebugSprite sprite = DebugSprite::diamond;
};

struct DebuggerAddPoly {
    bool add(const PathOpsV0Lib::Curve& );
    void add(const DebugOutput& );
    bool add(OpPoint );
    void add(const OpEdge* );
    void add(const OpIntersection* );  // point only
    void add(const OpSegment* );
    void add(const OpContour* );

    DebuggerState* debuggerState = nullptr;
    DebuggerWindow* window = nullptr;
    OpType opType;
    bool continueCurve = false;  // true if contour extends loop
    bool addingFill = false;  // true if added is fill, false if added is frame
    bool monotonic = false;
    bool pointOnly = false;
};

struct DebuggerPoly {
    void dump() const;
    void validate() const;

    PathOpsV0Lib::Curve c;
    PathOpsV0Lib::CurveData cData;  // used by construction lines
    static constexpr float fill_thickness = 0;
    std::vector<OpPoint> local;    // lines used to draw, in local coordinates
    std::vector<OpPoint> device;    // lines used to draw, in device coordinates
    std::vector<size_t> contours;  // index for each device contour
    OpType opType;
    float thickness = 1;    // special value for fill
    uint32_t color = black;
    float tStart = 0;
    float tEnd = 1;
    bool isPrimary = false;
};

struct OpDebugText {
#if OP_DEBUG_DUMP
    void dump(DebuggerWindow& ) const;
#endif
    OpType opType;
    OpPoint pt;    // in device coordinates
    OpPoint debugLocal;
    size_t cacheIndex;
    bool vertical = false;
    bool clip = false;
};

#endif
