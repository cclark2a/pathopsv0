#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "debugger/OpDebugPicture.h"

#define DRAW_SDL 1

bool OpDebugSkipBreak() {
    return true;
}

static TTF_Font* font = nullptr;
// bool debugUseAlt = false;

static SDL_Color toSDLColor(uint32_t c) {
    SDL_Color sdlColor = { (c >> 16) & 0xFF, (c >> 8) & 0xFF, (c >> 0) & 0xFF, (c >> 24) & 0xFF };
    return sdlColor;
}

size_t Window::addText(std::string str, uint32_t color) {
    auto found = std::find_if(debugPicture.textCache.begin(), debugPicture.textCache.end(), 
            [str, color](NativeTextCache& cache) {
            return str == cache.str && color == cache.color; } );
    if (debugPicture.textCache.end() != found)
        return found - debugPicture.textCache.begin();
    SDL_Color sdlColor = toSDLColor(color);
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, str.c_str(), 0, sdlColor);
    if (!textSurface) {
        OpDebugOut(std::string("Couldn't create text: %s\n") + SDL_GetError());
        exit(1);
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_DestroySurface(textSurface);
    size_t cacheIndex = debugPicture.textCache.size();
    OpVector size;
    SDL_GetTextureSize(texture, &size.dx, &size.dy);
    debugPicture.textCache.push_back({str, size, texture, color});
    return cacheIndex;
}

SDL_AppResult Window::draw() {
    if (!buffer)
        return SDL_APP_CONTINUE;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    char* pix;
    int pitch;
    SDL_LockTexture(polysTexture, nullptr, (void**)&pix, &pitch);
    int windowWidth, windowHeight;
    if (!SDL_GetWindowSize(window, &windowWidth, &windowHeight)) {
        OpDebugOut("failed to get window size: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_CONTINUE;
    }
    pentrek_draw(pix, windowWidth, windowHeight, pitch);
    SDL_UnlockTexture(polysTexture);  
    SDL_RenderTexture(renderer, polysTexture, nullptr, nullptr);
    for (OpDebugText& text : debugPicture.texts) {
        OP_ASSERT(text.cacheIndex < debugPicture.textCache.size());
        NativeTextCache& cache = debugPicture.textCache[text.cacheIndex];
        SDL_Texture* texture = (SDL_Texture*) cache.texture;
        SDL_FRect dst { text.pt.x, text.pt.y, cache.size.dx, cache.size.dy };
        if (text.vertical) {
            SDL_FPoint center { 0, 0 };
            SDL_RenderTextureRotated(renderer, texture, nullptr, &dst, -90.0, &center, 
                    SDL_FLIP_NONE);
        } else
            SDL_RenderTexture(renderer, texture, nullptr, &dst);
    }
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

void Window::drawText() {
    for (OpDebugText& text : debugPicture.texts) {
        OP_ASSERT(text.cacheIndex < debugPicture.textCache.size());
        const NativeTextCache& cache = debugPicture.getCache(text.cacheIndex);
        SDL_Texture* texture = (SDL_Texture*) cache.texture;
        SDL_FRect dst { text.pt.x, text.pt.y, cache.size.dx, cache.size.dy };
        if (text.vertical) {
            SDL_FPoint center { 0, 0 };
            SDL_RenderTextureRotated(renderer, texture, nullptr, &dst, -90.0, &center, 
                    SDL_FLIP_NONE);
        } else
            SDL_RenderTexture(renderer, texture, nullptr, &dst);
    }
}

void Window::update(Window& text, const char* filename) {
    if (debugPicture.update(*this, filename))
        text.debugPicture.copy(debugPicture);
}

SDL_AppResult Window::init(WindowEventHandler handler, std::string n, OpVector offset) {
    eventHandler = handler;
    name = n;
    SDL_Color color = { 0, 0, 0, SDL_ALPHA_OPAQUE };
    const int WINDOW_WIDTH = 1000;
    const int WINDOW_HEIGHT = 1000;
    if (!SDL_CreateWindowAndRenderer(("V0 Debugger " + name).c_str(), WINDOW_WIDTH, WINDOW_HEIGHT, 
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN, &window, &renderer)) {
        OpDebugOut("Couldn't create window and renderer: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    int x, y;
    if (SDL_GetWindowSize(window, &x, &y))
        windowSize = { (float) x, (float) y };
    if (SDL_GetWindowPosition(window, &x, &y)) {
        SDL_SetWindowPosition(window, (int) (x + offset.dx), (int) (y + offset.dy));
    }
    if (!SDL_ShowWindow(window)) {
        OpDebugOut("Couldn't show window at (" + STR(x) + ", " + STR(y) + "with offset "
                + offset.debugDump(DebugLevel::normal, DebugBase::dec) + ": " 
                + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    buffer = (int*) malloc(WINDOW_WIDTH * WINDOW_HEIGHT * sizeof(int));
    polysTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, 
            SDL_TEXTUREACCESS_STREAMING,  WINDOW_WIDTH, WINDOW_HEIGHT);

    return SDL_APP_CONTINUE;
}

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
    DebuggerState* debuggerState = new DebuggerState();
    *appstate = debuggerState;

    drawIDsOn = true;  // !!! hardcode for testing
    drawEdgesOn = true;  // !!! hardcode for testing
    drawWindingsOn = false;  // !!! hardcode for testing
    drawValuesOn = true;  // !!! hardcode for testing
    drawGridOn = false; // !!! hardcode for testing
    drawPointsOn = true;
    drawFillOn = false;
    if (!TTF_Init()) {
        OpDebugOut("Couldn't initialise SDL_ttf: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    font = TTF_OpenFont("C:/Windows/Fonts/segoeui.ttf", 14);
    if (!font) {
        OpDebugOut("Couldn't open font: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    static OpPoint lastMouse;
    static bool dragging = false;
    DebuggerState* debuggerState = (DebuggerState*) appstate;
    DebuggerEvent debuggerEvent = debuggerState->addEvent(SDL_GetModState(), event->window.windowID);
    switch (event->type) {
        case SDL_EVENT_MOUSE_WHEEL: 
            debuggerEvent.wheel = event->wheel.y;
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            debuggerEvent.mouseAction = MouseAction::click;
            SDL_GetMouseState(&lastMouse.x, &lastMouse.y);
            debuggerEvent.mouse = lastMouse;
            dragging = true;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: {
            dragging = false;
            break;
        }
        case SDL_EVENT_MOUSE_MOTION: {
            if (!debuggerEvent.focused)
                break;
            SDL_GetMouseState(&debuggerEvent.mouse.x, &debuggerEvent.mouse.y);
            if (lastMouse == debuggerEvent.mouse)
                break;
            if (dragging) {
                debuggerEvent.mouseDown = lastMouse;
                debuggerEvent.mouseAction = MouseAction::drag;
            } else
                debuggerEvent.mouseAction = MouseAction::move;
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (!debuggerEvent.focused)
                break;
            switch (uint8_t key = event->key.key) {
                case SDLK_LEFT:
                    debuggerEvent.key = (uint8_t) KeyCode::leftArrow;
                break;
                case SDLK_UP:
                    debuggerEvent.key = (uint8_t) KeyCode::upArrow;
                break;
                case SDLK_RIGHT:
                    debuggerEvent.key = (uint8_t) KeyCode::rightArrow;
                break;
                case SDLK_DOWN:
                    debuggerEvent.key = (uint8_t) KeyCode::downArrow;
                break;
                default:
                    if (KeyMods::none == debuggerEvent.keyMods) {
                        debuggerEvent.key = key;
                        break;
                    }
                    if (KeyMods::shift == debuggerEvent.keyMods) {
                        if ('a' <= key && key <= 'z') {
                            debuggerEvent.key = (uint8_t) key - 0x20;
                            break;
                        }
                        if ('0' <= key && key <= '9') {
                            debuggerEvent.key = ")!@#$%^&*("[key - '0'];
                            break;
                        }
                        const char* shifted =   "~!@#$%^&*()_+" "{}|"  ":\"" "<>?";
                        const char* unshifted = "`1234567890-=" "[]\\" ";'"  ",./";
                        static_assert(sizeof(shifted) == sizeof(unshifted));
                        for (int index = 0; index < strlen(unshifted); ++index) {
                            if (unshifted[index] == key) {
                                debuggerEvent.key = shifted[index];
                                break;
                            }
                        }
                    }
                break;
            }
        }
        debuggerEvent.doEvent();
        debuggerState->redraw();
    }
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    return SDL_APP_CONTINUE;
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
    DebuggerState* debuggerState = (DebuggerState*) appstate;
    static time_t lastTime = 0;
    struct stat info;
#if 1
    const char* opFileName = "d:/gerrit/skia/out/Debug/obj/dmp.txt";
#else
    const char* opFileName = "c:/users/cclar/source/repos/v0/v0/dmp2.txt";
#endif
    if (stat(opFileName, &info) == -1) {
        assert(0);
        return SDL_APP_FAILURE;
    }
    if (info.st_mtime != lastTime) {
        debuggerState->update(opFileName);
        lastTime = info.st_mtime;
    }
    debuggerState->draw();
    return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
    if (font)
        TTF_CloseFont(font);
    TTF_Quit();
}
