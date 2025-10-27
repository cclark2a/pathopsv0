// (c) 2025, Cary Clark cclark2@gmail.com
#include "HelpWindow.h"
#include "DebuggerState.h"
#include "OpCurveCurve.h"

HelpWindow::HelpWindow(DebuggerState* state)
    : DebuggerWindow(state, WheelTarget::none) {
    screen = { 0, 0, 365, 1000 };
    if (SDL_APP_CONTINUE != (state->error = init("help", { -200, -200 })))
        OpDebugOut("Couldn't initialize help window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(window);
}

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel HelpWindow::event(const DebuggerEvent& event) {
    OP_ASSERT(debuggerState->lastFocus);
    return debuggerState->lastFocus->event(event);
}

// updated when picture and text window change focus
void HelpWindow::update() {
    clearWindow();
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    OpPoint localLocation(10, 10);
    DebuggerEvent event(debuggerState, (SDL_Keymod) 0, (SDL_WindowID) 0);
    auto nextLine = [&localLocation, detailFont, this](std::string str) {
        if (str.empty())
            return;
        OpDebugText& text = addText(str, localLocation, debugBlack, detailFont);
        const NativeTextCache& cache = getCache(text.cacheIndex);
        localLocation.y += cache.size.dy;
    };
    std::string shortCuts = "\x01\x02\x03\x04" 
            "aAbBcCdDeEfFgGhHiIjJkKlLmMnNoOpPqQrRsStTuUvVwWxXuyYzZ"
            "0123456789-~?";
    for (char c : shortCuts) {
        event.key = c;
        nextLine(debuggerState->keyEvent(event, KeyAction::show).s);
    }
}

