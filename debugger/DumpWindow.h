// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef DumpWindow_DEFINED
#define DumpWindow_DEFINED

#include "DebuggerWindow.h"

struct DumpWindow : public DebuggerWindow {
    DumpWindow(DebuggerState* state);
    DrawLevel click(const DebuggerEvent* event);
    DrawLevel event(const DebuggerEvent& ) override;
    void update();
};

#endif
