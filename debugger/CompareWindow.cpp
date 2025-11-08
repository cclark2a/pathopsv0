// (c) 2025, Cary Clark cclark2@gmail.com
#include "CompareWindow.h"
#include "DebuggerState.h"
#define TEST_RASTER 1
#include "OpDebugRaster.h"
#include <sys/stat.h>

CompareWindow::CompareWindow(DebuggerState* state)
    : DebuggerWindow(state, WheelTarget::none) {
    screen = { 0, 0, 365, 1000 };
    if (SDL_APP_CONTINUE != (state->error = init("compare", { -200, -200 })))
        OpDebugOut("Couldn't initialize compare window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(window);
}

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel CompareWindow::event(const DebuggerEvent& event) {
    OP_ASSERT(debuggerState->lastFocus);
    return debuggerState->lastFocus->event(event);
}

bool CompareWindow::readBits() {
start here;
    return true;
}

void CompareWindow::update() {
    clearWindow();
    struct stat info;
    std::string filename = dmpFileToPath(BitsFile);
    if (stat(filename.c_str(), &info) == -1) 
        return;
    if (info.st_mtime != debuggerState->lastTime) {
        if (readBits()) {
            lastTime = info.st_mtime;
            return;
        } 
        if (updateAttempts > maxUpdateAttempts) {
            OpDebugOut("failed to update\n"); 
            OP_ASSERT(0);
            readBits();  // for debugging
            return ;
        }
    }

}
