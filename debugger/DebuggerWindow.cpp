// (c) 2025, Cary Clark cclark2@gmail.com

#include "OpDebugPicture.h"
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_init.h>

int keyModMultiplier(KeyMods mods) {
    int scale = 1;
    if (KeyMods::shift == (KeyMods::shift & mods))
        scale = 2;
    if (KeyMods::ctrl == (KeyMods::ctrl & mods))
        scale *= 4;
    if (KeyMods::alt == (KeyMods::alt & mods))
        scale *= 16;
    return scale;
}

DebuggerEvent::DebuggerEvent(SDL_Keymod mod, SDL_WindowID windowID) {
    if (SDL_KMOD_SHIFT & mod)
        keyMods |= KeyMods::shift;
    if (SDL_KMOD_CTRL & mod)
        keyMods |= KeyMods::ctrl;
    if (SDL_KMOD_ALT & mod)
        keyMods |= KeyMods::alt;
}

Window::Window() {
    debugPicture.window = this;
}

DebuggerState::DebuggerState() {
    SDL_AppResult result = text.init(textEvent, "text", { -100, -100 });
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise text window: " + std::string(SDL_GetError()) + "\n");
    }
    result = picture.init(pictureEvent, "picture", { 100, 100 } );
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise picture window: " + std::string(SDL_GetError()) + "\n");
    }
}

DebuggerEvent DebuggerState::addEvent(SDL_Keymod mod, SDL_WindowID windowID) {
    DebuggerEvent result(mod, windowID);
    result.focused = focus(windowID);
    return result;
}

void DebuggerEvent::doEvent() {
    if (!focused)
        return;
    if (wheel) {
        int scale = keyModMultiplier(keyMods);
        focused->debugPicture.zoom(wheel * scale);
        OpDebugOut("zoom:" + STR(focused->debugPicture.zoomFactor)
            + " wheel:" + STR(wheel)
            + " scale:" + STR(scale) + "\n");
        return;
    }
    if (MouseAction::drag == mouseAction) {
        focused->debugPicture.move(mouse - mouseDown);
        return;
    }
    (*focused->eventHandler)(focused, *this);
}

void DebuggerState::draw() {
    picture.draw();
    text.draw();
}

Window* DebuggerState::focus(SDL_WindowID id) {
    return SDL_GetWindowID(picture.window) == id ? &picture :
            SDL_GetWindowID(text.window) == id ? &text : nullptr;
}

void DebuggerState::redraw() {
    picture.debugPicture.redraw();
    text.debugPicture.redraw();
}

void DebuggerState::update(const char* filename) {
    picture.update(text, filename);
}