// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TextWindow_DEFINED
#define TextWindow_DEFINED

#include "DebuggerWindow.h"

struct TextWindow;

typedef DrawLevel (*TextAction)(const DebuggerEvent* , TextWindow* , OpType& );

struct TextWindow : public DebuggerWindow {
    TextWindow(DebuggerState* state);
    DebuggerPoly& addIdBox(const OpRect& , std::string , uint32_t color);
    int canScroll() const;
    DrawLevel doType(TextAction , const DebuggerEvent* );
    DrawLevel doWheel(const DebuggerEvent& , int delta) override;
    DrawLevel event(const DebuggerEvent& ) override;
    void playback(const char*& str ) override;
    std::string record() override;
    DrawLevel scroll(int );
    void update();
    void innerUpdate(int& );
    
    TTF_Font* detailFont = nullptr;
    OpVector boxWH { 50, 20 };  // !!! get metrics from font instead of hard-coding them
    std::string test;
    time_t lastTime;
    int lineHeight = 0;  // !!! change this to the text line height
    int detailHeight = 0;  // the height of the window content, visible or not
 //   int detailPos = 0;  // the offset into the window content (to make part of it visible)
    int scrollPos = 0;  // accumulated scroll wheel value, pinned to detail height - window pane
    int maxUpdateAttempts = 16;
    int updateAttempts = 0;
    bool showAll = false;
    bool showAliases = false;
    bool showCurveCurve = false;
    bool showFull = false;
    bool showEdgeHulls = false;
    bool showJoin = false;
    bool showLinks = false;
    bool showPoints = false;
    bool showRays = false;
    bool showTest = false;
    bool showTree = false;
};

#endif
