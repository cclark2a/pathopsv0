// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef TextWindow_DEFINED
#define TextWindow_DEFINED

#include "DebuggerWindow.h"

typedef DrawLevel (*EventAction)(const DebuggerEvent* , struct TextWindow* , OpType& );

struct TextWindow : public DebuggerWindow {
    TextWindow(DebuggerState* state);
    DebuggerPoly& addIdBox(const OpRect& , std::string , uint32_t color);
    DrawLevel doType(EventAction , const DebuggerEvent* );
    DrawLevel event(const DebuggerEvent& ) override;
    void playback(const char*& str ) override;
    std::string record() override;
    DrawLevel scroll(int );
    void update();
    void innerUpdate(int& );
    
    TTF_Font* detailFont = nullptr;
    OpVector boxWH { 50, 20 };  // !!! get metrics from font instead of hard-coding them
    int lineHeight = 0;  // !!! change this to the text line height
    int detailHeight = 0;  // the height of the window content, visible or not
 //   int detailPos = 0;  // the offset into the window content (to make part of it visible)
    int scrollPos = 0;  // accumulated scroll wheel value, pinned to detail height - window pane
    bool showAll = false;
    bool showAliases = false;
    bool showCurveCurve = false;
    bool showFull = false;
    bool showEdgeHulls = false;
    bool showJoin = false;
    bool showLinks = false;
    bool showPoints = false;
    bool showRays = false;
    bool showTree = false;
};

#endif
