// (c) 2025, Cary Clark cclark2@gmail.com
#include "DumpWindow.h"
#include "DebuggerState.h"
#include "OpContext.h"

DumpWindow::DumpWindow(DebuggerState* state)
    : DebuggerWindow(state, WheelTarget::none) {
    screen = { 0, 0, 365, 1000 };
    if (SDL_APP_CONTINUE != (state->error = init("dump", { -200, -200 })))
        OpDebugOut("Couldn't initialize dump window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(window);
}

DrawLevel DumpWindow::click(const DebuggerEvent* event) {
    int windowWidth, windowHeight;
    if (!SDL_GetWindowSize(window, &windowWidth, &windowHeight)) {
        OpDebugOut("failed to get window size: " + std::string(SDL_GetError()) + "\n");
        return DrawLevel::none;
    }
    OpPoint localLocation(10, 10);
    float lineHeight = debuggerState->textWindow.boxWH.dy;
    if (event->mouse.y < localLocation.y)
        return DrawLevel::none;
    int index = (int) ((event->mouse.y - localLocation.y) / lineHeight);
    if (index >= (int) debuggerState->dumps.size())
        return DrawLevel::none;
    if (debuggerState->currentDump == index)
        return DrawLevel::none;
    debuggerState->saveSelection();
    debuggerState->currentDump = index;
    return DrawLevel::file;  //  preserve selected, switch, and restore selected
}

DrawLevel DumpWindow::event(const DebuggerEvent& debuggerEvent) {    
    if (DrawLevel common = debuggerState->eventCommon(debuggerEvent); DrawLevel::none != common)
        return common;
    if (MouseAction::click == debuggerEvent.mouseAction) 
        return click(&debuggerEvent);
    return DrawLevel::none;
}

void DumpWindow::update() {
    clearWindow();
    OpPoint localLocation(10, 10);
    OpRect r { 0, 
            localLocation.y + debuggerState->textWindow.boxWH.dy * debuggerState->currentDump, 
            screen.right, 
            localLocation.y + debuggerState->textWindow.boxWH.dy * (debuggerState->currentDump + 1) 
    };
    add(r, yellow, 0);
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    auto nextLine = [&localLocation, detailFont, this](std::string str) {
        addText(str, localLocation, debugBlack, detailFont);
        localLocation.y += debuggerState->textWindow.boxWH.dy;
    };
    for (DebuggerDump& dump : debuggerState->dumps) {
        if (!dump.context)
            nextLine("!!! missing context");
        else
            nextLine(dump.context->debugFilename + ": " + dump.context->debugDescription);
    }
}
