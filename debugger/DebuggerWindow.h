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
    bool add(const OpCurve& , DebuggerAddPoly* );
    DebuggerPoly& add(const OpRect& , uint32_t color, float thickness);
    bool add(OpPoint , OpPoint , DebuggerAddPoly* );
    void add(std::vector<OpPoint>& points );
    void addLine(OpPoint pt1, OpPoint pt2);
    SDL_AppResult addFont(float fontSize, TTF_Font** result = nullptr);
    OpDebugText& addClipped(std::string , OpPoint , uint32_t color, TTF_Font* f = nullptr);
    size_t addText(std::string , uint32_t color, TTF_Font* f = nullptr);
    OpDebugText& addText(std::string , OpPoint , uint32_t color, TTF_Font* = nullptr, 
            bool rotated = false);
    SDL_AppResult allocateBuffers();
    void append(OpPoint );
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
    const NativeTextCache& getCache(size_t index) const;
    SDL_AppResult init(std::string name, OpVector offset);
    void pentrek_draw(char*, int width, int height, int pitch);
    virtual void playback(const char*&  str) { playbackCommon(str); }
    void playbackCommon(const char*& );
    virtual std::string record() { return recordCommon(); };
    std::string recordCommon();
    void setSize();
    // self-debugging:
    std::string debugTextDump(size_t index);
    void dumpWindow();
    void validate() const;

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
    int fontSize = TEXT_FONT_SIZE;
    int windowID = 0;
    WheelTarget wheelTarget; 
};

#endif
