// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef CompareWindow_DEFINED
#define CompareWindow_DEFINED

#include "DebuggerWindow.h"

struct DebugRaster;

// !!! eventually move this into debugging strings in curves or maybe debugger bits txt
extern const std::vector<std::string> drawCompareStrs;

struct CompareWindow : public DebuggerWindow {
    CompareWindow(DebuggerState* state);
    SDL_AppResult draw() override;
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    bool readBits();
    void update();  // draw help corresponding to topmost window
    DebugRaster* debugRaster = nullptr;
    OpRect leftFocus;
    OpRect rightFocus;
    time_t lastTime = 0;
    float scale = OpNaN;
    float xOffset = OpNaN;
    float yOffset = OpNaN;
    int updateAttempts = 0;
    int maxUpdateAttempts = 16;
    int margin = 1;
    int leftBits = 0;
    int rightBits = 1;
};

#endif
