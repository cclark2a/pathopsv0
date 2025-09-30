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
    auto found = std::find_if(textCache.begin(), textCache.end(), 
            [str, color](NativeTextCache& cache) {
            return str == cache.str && color == cache.color; } );
    if (textCache.end() != found)
        return found - textCache.begin();
    SDL_Color sdlColor = toSDLColor(color);
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, str.c_str(), 0, sdlColor);
    if (!textSurface) {
        OpDebugOut(std::string("Couldn't create text: %s\n") + SDL_GetError());
        exit(1);
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_DestroySurface(textSurface);
    size_t cacheIndex = textCache.size();
    OpVector size;
    SDL_GetTextureSize(texture, &size.dx, &size.dy);
    textCache.push_back({str, size, texture, color});
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
    drawText();
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

void Window::drawText() {
    for (OpDebugText& text : texts) {
        OP_ASSERT(text.cacheIndex < textCache.size());
        const NativeTextCache& cache = getCache(text.cacheIndex);
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

SDL_AppResult Window::init(std::string n, OpVector offset) {
    name = n;
    const int WINDOW_WIDTH = 1000;
    const int WINDOW_HEIGHT = 1000;
    if (!SDL_CreateWindowAndRenderer(("V0 Debugger " + name).c_str(), WINDOW_WIDTH, WINDOW_HEIGHT, 
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN, &window, &renderer)) {
        OpDebugOut("Couldn't create window and renderer: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    windowID = SDL_GetWindowID(window);
    setSize();
    int x, y;
    if (SDL_GetWindowPosition(window, &x, &y))
        SDL_SetWindowPosition(window, (int) (x + offset.dx), (int) (y + offset.dy));
    if (!SDL_ShowWindow(window)) {
        OpDebugOut("Couldn't show window " + n + " at (" + STR(x) + ", " + STR(y) + "with offset "
                + offset.debugDump(DebugLevel::normal, DebugBase::dec) + ": " 
                + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    allocateBuffers(WINDOW_WIDTH, WINDOW_HEIGHT);
    return SDL_APP_CONTINUE;
}

SDL_AppResult Window::allocateBuffers(int width, int height) {
    size_t bufferSize = width * height * sizeof(uint32_t);
    if (buffer)
        free(buffer);
    buffer = (int*) malloc(bufferSize);
    if (!buffer) {
        OpDebugOut("Couldn't allocate buffer of size: " + STR(bufferSize) + "\n");
        return SDL_APP_FAILURE;
    }
    memset(buffer, 0xFF, bufferSize);
    if (polysTexture)
        SDL_DestroyTexture(polysTexture);
    polysTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, 
            SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!polysTexture) {
        OpDebugOut("Couldn't allocate texture w/h: " + STR(width) + "/" + STR(height) + "\n");
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
    DebuggerState* debuggerState = new DebuggerState();
    if (argc == 2)
        debuggerState->opFileName = argv[1];
    *appstate = debuggerState;
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
    static OpPoint downMouse;
    static OpPoint lastMouse;
    static bool dragging = false;
    DebuggerState* debuggerState = (DebuggerState*) appstate;
    DebuggerEvent debuggerEvent(debuggerState, SDL_GetModState(), event->window.windowID);
    bool verbose = false;
    bool systemRedraw = false;
    std::string windowName;
    if (debuggerState->pictureWindow.windowID == event->window.windowID)
        windowName = debuggerState->pictureWindow.name;
    else if (debuggerState->textWindow.windowID == event->window.windowID)
        windowName = debuggerState->textWindow.name;
    if (       event->type != SDL_EVENT_MOUSE_MOTION         // 0x400
            && event->type != SDL_EVENT_WINDOW_SHOWN         // 0x202
            && event->type != SDL_EVENT_WINDOW_EXPOSED       // 0x204
            && event->type != SDL_EVENT_WINDOW_MOVED         // 0x205
            && event->type != SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED // 0x207
            && event->type != SDL_EVENT_WINDOW_MOUSE_ENTER   // 0x20c
            && event->type != SDL_EVENT_WINDOW_MOUSE_LEAVE   // 0x20d
            && event->type != SDL_EVENT_WINDOW_FOCUS_GAINED  // 0x20e
            && event->type != SDL_EVENT_WINDOW_FOCUS_LOST    // 0x20f
            && event->type != SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED // 0x214
            && event->type != SDL_EVENT_CLIPBOARD_UPDATE     // 0x900
            )
        OpDebugOut("event:" + OpDebugIntToHex(event->type) + "\n");
    switch (event->type) {
        case SDL_EVENT_CLIPBOARD_UPDATE:
            OpDebugOut("clipboard update\n");
            break;
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            OpDebugOut(windowName + " pixel size changed\n");
            systemRedraw = true;
            break;
        case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
            OpDebugOut(windowName + " display scale changed\n");
            systemRedraw = true;
            break;
        case SDL_EVENT_WINDOW_SHOWN:
            OpDebugOut(windowName + " shown\n");
            systemRedraw = true;
            break;
        case SDL_EVENT_WINDOW_RESIZED:
            OpDebugOut(windowName + " resized\n");
            systemRedraw = true;
            break;
        case SDL_EVENT_WINDOW_EXPOSED:
            OpDebugOut(windowName + " exposed\n");
            systemRedraw = true;
            break;
        case SDL_EVENT_WINDOW_MOVED:
            OpDebugOut(windowName + " moved\n");
            break;
        case SDL_EVENT_WINDOW_MOUSE_ENTER:
            if (verbose) OpDebugOut(windowName + " mouse enter\n");
            ;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            if (verbose) OpDebugOut(windowName + " mouse leave\n");
            ;
            break;
        case SDL_EVENT_WINDOW_FOCUS_GAINED:
            OpDebugOut(windowName + " focus gained\n");
            ;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            OpDebugOut(windowName + " focus lost\n");
            ;
            break;
        case SDL_EVENT_MOUSE_WHEEL: 
            debuggerEvent.wheel = event->wheel.y;
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            debuggerEvent.mouseAction = MouseAction::click;
            SDL_GetMouseState(&downMouse.x, &downMouse.y);
            debuggerEvent.mouse = downMouse;
            lastMouse = downMouse;
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
                debuggerEvent.mouseDown = downMouse;
                debuggerEvent.mouseLast = lastMouse;
                lastMouse = debuggerEvent.mouse;
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
    }
    DrawLevel update = debuggerEvent.doEvent();
    if (DrawLevel::file == update)
        debuggerState->update();
    if (DrawLevel::update == update || systemRedraw)
        debuggerState->redraw();
    else if (DrawLevel::draw == update)
        debuggerState->draw();
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    return SDL_APP_CONTINUE;
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
    DebuggerState* debuggerState = (DebuggerState*) appstate;
    static time_t lastTime = 0;
    struct stat info;
    if (stat(debuggerState->opFileName.c_str(), &info) == -1) {
        OP_ASSERT(0);
        return SDL_APP_FAILURE;
    }
    if (info.st_mtime != lastTime) {
        debuggerState->update();
        lastTime = info.st_mtime;
    }
    return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
    if (font)
        TTF_CloseFont(font);
    TTF_Quit();
}
