// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DebuggerWindow_DEFINED
#define DebuggerWindow_DEFINED

#include "DebuggerTypes.h"
#include <SDL3/SDL_init.h>

struct DebuggerAddPoly;
struct DebuggerEvent;
struct DebuggerPoly;
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

struct DebuggerWindow {
    DebuggerWindow(DebuggerState* state, WheelTarget);
    void add(const OpCurve& , DebuggerAddPoly* );
    DebuggerPoly& add(const OpRect& , uint32_t color, float thickness);
    void add(OpPoint , OpPoint , DebuggerAddPoly* );
    void add(std::vector<OpPoint>& points );
    void addLine(OpPoint pt1, OpPoint pt2);
    SDL_AppResult addFont(float fontSize, TTF_Font** result = nullptr);
    OpDebugText& addClipped(std::string , OpPoint , uint32_t color, TTF_Font* f = nullptr);
    size_t addText(std::string , uint32_t color, TTF_Font* f = nullptr);
    OpDebugText& addText(std::string , OpPoint , uint32_t color, TTF_Font* = nullptr, 
            bool rotated = false);
    SDL_AppResult allocateBuffers(int width, int height);
    void append(OpPoint );
    void clearWindow();
    OpContext* context();
    void deleteTextCache();
    virtual DrawLevel doWheel(const DebuggerEvent& , int delta) { return DrawLevel::none; }
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
    OpVector threshold;
    float pixelScale = 1;  // if this is non-square, much work will need to be done...
    float topClip = 0;  // if non-zero, clip text above this vertical offset
    int fontSize = 14;
    int windowID = 0;
    WheelTarget wheelTarget; 
};

#endif
