// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef CompareWindow_DEFINED
#define CompareWindow_DEFINED

#include "DebuggerWindow.h"

struct DebugRaster;

struct CompareWindow : public DebuggerWindow {
    CompareWindow(DebuggerState* state);
    SDL_AppResult draw() override;
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    bool readBits();
    void update();  // draw help corresponding to topmost window
    DebugRaster* debugRaster = nullptr;
    time_t lastTime = 0;
    int updateAttempts = 0;
    int maxUpdateAttempts = 16;
    int margin = 8;
};

#endif
