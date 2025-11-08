// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef CompareWindow_DEFINED
#define CompareWindow_DEFINED

#include "DebuggerWindow.h"

struct CompareWindow : public DebuggerWindow {
    CompareWindow(DebuggerState* state);
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    bool readBits();
    void update();  // draw help corresponding to topmost window
    time_t lastTime;
    int updateAttempts = 0;
    int maxUpdateAttempts = 16;
};

#endif
