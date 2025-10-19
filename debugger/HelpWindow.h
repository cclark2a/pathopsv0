// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef HelpWindow_DEFINED
#define HelpWindow_DEFINED

#include "DebuggerWindow.h"

struct HelpWindow : public DebuggerWindow {
    HelpWindow(DebuggerState* state)
        : DebuggerWindow(state) {
        screen = { 0, 0, 365, 1000 };
    }
    DrawLevel event(const DebuggerEvent& ) override;  // defer to topmost window
    void update();  // draw help corresponding to topmost window
};

#endif
