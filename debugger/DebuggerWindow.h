// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DebuggerWindow_DEFINED
#define DebuggerWindow_DEFINED

#include "DebuggerTypes.h"
#include <SDL3/SDL_init.h>

struct DebuggerAddPoly;
struct DebuggerEvent;
struct DebuggerPoly;
struct OpContext;
struct OpCurve;
struct OpDebugText;
struct NativeTextCache;
struct SDL_Window;
struct SDL_Renderer;

enum class WheelTarget {
    none,
    zoomAndKeyPan,  // picture only
    zoomAndKeyZoom, // picture only
    threshold,      // picture only
    font,           // picture and text
    scroll         // text only
    // !!! add grid lines?
    // !!! add detail font for text window?
};

extern const std::array<std::string, 6> WheelTargetVerbage;

struct Bumper {
    void bumpUp(int delta) {
        lastIndex = next(delta);
    }

    std::string label() {
        return labelAt(lastIndex);
    }

    int next(int delta) {
        int result = lastIndex + delta;
        if (result < 0)
            result = size() - 1;
        else if (result >= size())
            result = 0;
        return result;
    }

    std::string nextLabel(int delta) {
        return labelAt(next(delta));
    }
    
    virtual std::string labelAt(int index) = 0;
    virtual int size() const = 0;

    int lastIndex = 0;
};

struct DebuggerWindow {
    DebuggerWindow(DebuggerState* state, WheelTarget);
    void add(const OpCurve& , DebuggerAddPoly* , float tStart, float tEnd,
            float cStart, float cEnd);
    DebuggerPoly& add(const OpRect& , uint32_t color, float thickness);
    void add(DebuggerAddPoly* , const OpPtT& );
    void add(DebuggerAddPoly* , const OpPtT& , const OpPtT&  
            CLIP_PARAM(const OpCurve& ,float cStart, float cEnd));
    void add(std::vector<OpPoint>& points );
    void addLine(OpPoint pt1, OpPoint pt2);
    SDL_AppResult addFont(float fontSize, TTF_Font** result = nullptr);
    OpDebugText& addClipped(std::string , OpPoint , uint32_t color, TTF_Font* f = nullptr);
    size_t addText(std::string , uint32_t color, TTF_Font* f = nullptr);
    OpDebugText& addText(std::string , OpPoint , uint32_t color, TTF_Font* = nullptr, 
            bool rotated = false);
    SDL_AppResult allocateBuffers();
    void append(std::vector<DebuggerPoly>& , OpDPoint );
    void clearWindow();
    OpContext* context();
    void deleteTextCache();
    virtual DrawLevel doWheel(const DebuggerEvent& , int delta) { return DrawLevel::none; }
    virtual SDL_AppResult draw() { return drawCommon(); }
    SDL_AppResult drawCommon();
    virtual bool drawOne(DebuggerPoly& ) { return true; }
    void drawText();
    virtual DrawLevel event(const DebuggerEvent& ) { return DrawLevel::none; }
    DebuggerPoly* findPoly(const OpEdge* );
    DebuggerPoly* findPoly(const OpSegment* );
    DebuggerPoly* findPolyByID(int );
    DebuggerPoly* findRectByID(int );
    std::vector<DebuggerPoly>& findPolys(OpType );
    const NativeTextCache& getCache(size_t index) const;
    SDL_AppResult init(std::string name, OpVector offset);
    void pentrek_draw(char*, int width, int height, int pitch);
    virtual void playback(const char*&  str) { playbackCommon(str); }
    void playbackCommon(const char*& );
    virtual std::string record() { return recordCommon(); };
    std::string recordCommon();
    void setSize();
    std::vector<std::vector<DebuggerPoly>*> tangentPolys();
    OpPoint toLocal(OpPoint p) const;
    OpDPoint toLocal(OpDPoint p) const;
    OpPoint toDevice(OpPoint p) const;
    OpDPoint toDevice(OpDPoint p) const;

#if OP_DEBUG
    // self-debugging:
    std::string debugTextDump(size_t index);
    std::string debugDump(DebugLevel, DebugBase) const;
#endif
#if DEBUG_CLIP
    DebugClip* createDebugClip(OpType& , const OpCurve& , float tStart, float tEnd);
    void findDebugClips(OpType& , float tStart, float tEnd, std::vector<DebugClip*>* );
#endif
#if OP_DEBUG_VALIDATE
    void validate() const;
#endif

    DebuggerState* debuggerState;
    DebuggerAddPoly addPoly;
    std::array<std::vector<DebuggerPoly>*, 7> allPolys { &edges, &contours, &intersections, &segments,
            &rects, &polyPoints, &lines };
    std::array<std::vector<DebuggerPoly>*, 4> polyIDs { &edges, &contours, &intersections, &segments };
    std::array<std::vector<DebuggerPoly>*, 2> touchIDs { &edges, &segments };
    std::vector<DebuggerPoly> edges;
    std::vector<DebuggerPoly> contours;
    std::vector<DebuggerPoly> intersections;
    std::vector<DebuggerPoly> segments;
    std::vector<DebuggerPoly> output;
    std::vector<DebuggerPoly> rects;
    std::vector<DebuggerPoly> polyPoints;
    std::vector<DebuggerPoly> lines;
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
    OpDVector threshold;
    double scale = 1; // factor to go from local to device
    float pixelScale = 1;  // if this is non-square, much work will need to be done...
    float topClip = 0;  // if non-zero, clip text above this vertical offset
    int fontSize = TEXT_FONT_SIZE;
    int windowID = 0;
    WheelTarget wheelTarget; 
#if DEBUG_CLIP
    std::vector<DebugClip> debugClips;
#endif
};

#endif
