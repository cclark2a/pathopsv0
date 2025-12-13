#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "DebuggerState.h"

SDL_AppResult DebuggerWindow::addFont(float fontSize, TTF_Font** result) {
    if (!result)
        result = &font;
    float multiplier = pixelScale;  // !!! for now, just pick one
    fontSize *= multiplier;
    TTF_Font* oldFont = *result;
    if (oldFont) {
        for (auto& cache : textCache) {
            if (cache.window == this && cache.font == oldFont)      
                SDL_DestroyTexture(cache.texture);
        }
        auto& v = textCache;
        v.erase(std::remove_if(v.begin(), v.end(), [oldFont, this](const NativeTextCache& c) { 
                return c.window == this && c.font == oldFont; }), v.end());
        TTF_CloseFont(oldFont);
    }
#ifdef _WIN32
    *result = TTF_OpenFont("C:/Windows/Fonts/segoeui.ttf", fontSize);
#elif defined __APPLE__
    *result = TTF_OpenFont("/System/Library/Fonts/Monaco.ttf", fontSize);
#else
    return Fail("missing font for platform");
#endif
    if (!*result)
        return Fail("Couldn't open font");
    return SDL_APP_CONTINUE;
}

size_t DebuggerWindow::addText(std::string str, uint32_t color, TTF_Font* f) {
    if (!f)
        f = font;
    auto found = std::find_if(textCache.begin(), textCache.end(), 
            [f, str, color](NativeTextCache& cache) {
            return f == cache.font && str == cache.str && color == cache.color; } );
    if (textCache.end() != found)
        return found - textCache.begin();
    auto toSDLColor = [](uint32_t c) {
        auto byte = [c](int component) { return (uint8_t) (c >> (component * 8)); };
        SDL_Color sdlColor = { byte(2), byte(1), byte(0), byte(3) };
        return sdlColor;
    };
    SDL_Color sdlColor = toSDLColor(color);
    SDL_Surface* textSurface = TTF_RenderText_Blended_Wrapped(f, str.c_str(), 0, sdlColor, 0);
    if (!textSurface) {
        ReportError("Couldn't create text:\"" + str + "\"\n");
        exit(1);
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_DestroySurface(textSurface);
    size_t cacheIndex = textCache.size();
    OpVector size;
    SDL_GetTextureSize(texture, &size.dx, &size.dy);
    size /= pixelScale;
    textCache.push_back({f, this, str, size, texture, color});
    return cacheIndex;
}

SDL_AppResult DebuggerWindow::allocateBuffers() {
    int width, height;
    if (!SDL_GetWindowSize(window, &width, &height))
        return Continue("failed to get window size");
    size_t bufferSize = width * height * sizeof(uint32_t);
    if (buffer)
        free(buffer);
    buffer = (int*) malloc(bufferSize);
    if (!buffer)
        return Fail("Couldn't allocate buffer of size: " + STR(bufferSize));
    memset(buffer, 0xFF, bufferSize);
    if (polysTexture)
        SDL_DestroyTexture(polysTexture);
    polysTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, 
            SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!polysTexture)
        return Fail("Couldn't allocate texture w/h: " + STR(width) + "/" + STR(height));
    return SDL_APP_CONTINUE;
}

void DebuggerWindow::deleteTextCache() {
    for (auto& entry : textCache) {
        SDL_DestroyTexture(entry.texture);
    }
    textCache.clear();
}

SDL_AppResult DebuggerWindow::drawCommon() {
    if (!buffer)
        return SDL_APP_CONTINUE;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    char* pix;
    int pitch;
    SDL_LockTexture(polysTexture, nullptr, (void**)&pix, &pitch);
    int windowWidth, windowHeight;
    if (!SDL_GetWindowSize(window, &windowWidth, &windowHeight))
        return Continue("failed to get window size");
    pentrek_draw(pix, windowWidth, windowHeight, pitch);
    SDL_UnlockTexture(polysTexture);  
    SDL_RenderTexture(renderer, polysTexture, nullptr, nullptr);
    drawText();
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

void DebuggerWindow::drawText() {
    for (OpDebugText& text : texts) {
        OP_ASSERT(text.cacheIndex < textCache.size());
        const NativeTextCache& cache = getCache(text.cacheIndex);
        SDL_Texture* texture = (SDL_Texture*) cache.texture;
        SDL_FRect dst { text.pt.x, text.pt.y, cache.size.dx, cache.size.dy };
        dst.x *= pixelScale;
        dst.y *= pixelScale;
        dst.w *= pixelScale;
        dst.h *= pixelScale;
        if (text.vertical) {
            SDL_FPoint center { 0, 0 };
            SDL_RenderTextureRotated(renderer, texture, nullptr, &dst, -90.0, &center, 
                    SDL_FLIP_NONE);
            continue;
        }
        if (text.clip && topClip) {
            if (text.pt.y + cache.size.dy <= topClip)
                continue;
            float upperCut = topClip - text.pt.y;
            if (0 < upperCut) {
                float scaledCut = upperCut * pixelScale;
                SDL_FRect src { 0, scaledCut, dst.w, dst.h - scaledCut };
                dst.y = topClip * pixelScale;
                dst.h -= scaledCut;
                SDL_RenderTexture(renderer, texture, &src, &dst);
                continue;
            }
        }
        SDL_RenderTexture(renderer, texture, nullptr, &dst);
    }
}

SDL_AppResult DebuggerWindow::init(std::string n, OpVector offset) {
    name = n;
    std::string windowName = "V0 Debugger " + n;
    if (!SDL_CreateWindowAndRenderer(windowName.c_str(), screen.width(), screen.height(), 
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY, 
            &window, &renderer))
        return Fail("Couldn't create window and renderer");
    SDL_SetRenderLogicalPresentation(renderer, (int) screen.width(), (int) screen.height(),
            SDL_LOGICAL_PRESENTATION_DISABLED);
    windowID = SDL_GetWindowID(window);
    setSize();
    int x, y;
    if (SDL_GetWindowPosition(window, &x, &y))
        SDL_SetWindowPosition(window, (int) (x + offset.dx), (int) (y + offset.dy));
    if (!SDL_ShowWindow(window))
        return Fail("Couldn't show window " + n + " at (" + STR(x) + ", " + STR(y) + "with offset "
                + offset.debugDump(DebugLevel::normal, DebugBase::dec));
    allocateBuffers();
    int pixelsW, pixelsH;
    if (!SDL_GetWindowSizeInPixels(window, &pixelsW, &pixelsH))
        OpDebugOut(windowName + ": could not get size in pixels\n"); 
    else {
        OpVector scale((float) pixelsW / screen.width(), (float) pixelsH / screen.height());
        OP_ASSERT(scale.dx == scale.dy);  // !!! if screen is not square scale, debug
        pixelScale = scale.dx;
    }
    return SDL_APP_CONTINUE;
}

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
//    SDL_SetHint(SDL_HINT_VIDEO_HIGHDPI_DISABLED, "1");
    if (!TTF_Init())
        return Fail("Couldn't initialize SDL_ttf");
    DebuggerState* debuggerState = new DebuggerState();
    if (!debuggerState)
        return Fail("failed to allocate debuggerState");
    if (debuggerState->error)
        return Fail("debuggerState error: " + STR(debuggerState->error));
    *appstate = debuggerState;
    return SDL_APP_CONTINUE;
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    static OpPoint downMouse;
    static OpPoint lastMouse;
    static bool dragging = false;
    DebuggerState* state = (DebuggerState*) appstate;
    bool systemRedraw = false;
    // if event type can accumulate safely (window resize, window move, mouse wheel, mouse move)
    // defer until event queue is empty or another event type is seen
    uint32_t accumulate = 0;
    unsigned int winID = event->window.windowID;
    DebuggerEvent debuggerEvent(state, SDL_GetModState(), winID);
    DebuggerWindow* eventWindow = 
            state->pictureWindow.windowID == winID ? (DebuggerWindow*) &state->pictureWindow  
            : state->textWindow.windowID == winID ? (DebuggerWindow*) &state->textWindow 
            : state->helpWindow.windowID == winID ? (DebuggerWindow*) &state->helpWindow 
            : state->compareWindow.windowID == winID ? (DebuggerWindow*) &state->compareWindow 
            : state->dumpWindow.windowID == winID ? (DebuggerWindow*) &state->dumpWindow 
            : nullptr;
    std::string windowName = eventWindow ? eventWindow->name : "(unnamed window)";
    do {
        if (state->verboseLevel && event->type != SDL_EVENT_MOUSE_MOTION         // 0x400
                && event->type != SDL_EVENT_WINDOW_SHOWN         // 0x202
                && event->type != SDL_EVENT_WINDOW_EXPOSED       // 0x204
                && event->type != SDL_EVENT_WINDOW_MOVED         // 0x205
                && event->type != SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED // 0x207
                && event->type != SDL_EVENT_WINDOW_MOUSE_ENTER   // 0x20c
                && event->type != SDL_EVENT_WINDOW_MOUSE_LEAVE   // 0x20d
                && event->type != SDL_EVENT_WINDOW_FOCUS_GAINED  // 0x20e
                && event->type != SDL_EVENT_WINDOW_FOCUS_LOST    // 0x20f
                && event->type != SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED // 0x214
                && event->type != SDL_EVENT_KEY_DOWN // 0x300
                && event->type != SDL_EVENT_KEY_UP // 0x301
                && event->type != SDL_EVENT_CLIPBOARD_UPDATE     // 0x900
                )
                OpDebugOut("event:" + OpDebugIntToHex(event->type) + "\n");
        switch (event->type) {
            case SDL_EVENT_CLIPBOARD_UPDATE:
                if (state->verboseLevel) OpDebugOut("clipboard update\n");
                break;
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                if (state->verboseLevel) OpDebugOut(windowName + " pixel size changed\n");
                systemRedraw = true;
                break;
            case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
                if (state->verboseLevel) OpDebugOut(windowName + " display scale changed\n");
                systemRedraw = true;
                break;
            case SDL_EVENT_WINDOW_SHOWN:
                if (state->verboseLevel) OpDebugOut(windowName + " shown\n");
                systemRedraw = true;
                break;
            case SDL_EVENT_WINDOW_RESIZED:
                if (state->verboseLevel) OpDebugOut(windowName + " resized\n");
                accumulate = event->type;
                break;
            case SDL_EVENT_WINDOW_EXPOSED:
                if (state->verboseLevel) OpDebugOut(windowName + " exposed\n");
                systemRedraw = true;
                break;
            case SDL_EVENT_WINDOW_MOVED:
                if (state->verboseLevel) OpDebugOut(windowName + " moved\n");
                break;
            case SDL_EVENT_WINDOW_MOUSE_ENTER:
                if (state->verboseLevel > 1) OpDebugOut(windowName + " mouse enter\n");
                ;
                break;
            case SDL_EVENT_WINDOW_MOUSE_LEAVE:
                if (state->verboseLevel > 1) OpDebugOut(windowName + " mouse leave\n");
                ;
                break;
            case SDL_EVENT_WINDOW_FOCUS_LOST:
                if (state->verboseLevel) OpDebugOut(windowName + " focus lost\n");
                ;
                break;
            case SDL_EVENT_MOUSE_WHEEL: 
                debuggerEvent.wheel += event->wheel.y;
                break;
            case SDL_EVENT_WINDOW_FOCUS_GAINED:
                if (state->verboseLevel) OpDebugOut(windowName + " focus gained\n");
                if (!state->context)
                    break;
                [[fallthrough]];
            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                debuggerEvent.mouseAction = MouseAction::click;
                SDL_GetMouseState(&downMouse.x, &downMouse.y);
                debuggerEvent.mouse = downMouse;
                lastMouse = downMouse;
                dragging = SDL_EVENT_MOUSE_BUTTON_DOWN == event->type;
                break;
            }
            case SDL_EVENT_MOUSE_BUTTON_UP:
                dragging = false;
                break;
            case SDL_EVENT_MOUSE_MOTION:
                accumulate = event->type;
                break;
            case SDL_EVENT_KEY_DOWN: {
                if (!debuggerEvent.focused)
                    break;
                switch (SDL_Keycode key = event->key.key) {
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
        SDL_Event peek;
        if (1 != SDL_PeepEvents(&peek, 1, SDL_PEEKEVENT, SDL_EVENT_FIRST, SDL_EVENT_LAST)
                || peek.type != accumulate)
            break;
    } while (true);
    switch (accumulate) {
        case SDL_EVENT_WINDOW_RESIZED:
            eventWindow->setSize();
            systemRedraw = true;
        break;
        case SDL_EVENT_WINDOW_MOVED:
        break;
        case SDL_EVENT_MOUSE_WHEEL:
        break;
        case SDL_EVENT_MOUSE_MOTION: {
            if (!debuggerEvent.focused)
                break;
            SDL_GetMouseState(&debuggerEvent.mouse.x, &debuggerEvent.mouse.y);
            if (lastMouse == debuggerEvent.mouse)
                break;
            debuggerEvent.mouseLast = lastMouse;
            lastMouse = debuggerEvent.mouse;
            if (dragging) {
                debuggerEvent.mouseDown = downMouse;
                debuggerEvent.mouseAction = MouseAction::drag;
            } else
                debuggerEvent.mouseAction = MouseAction::move;
            break;
        }
    }
    DrawLevel update = debuggerEvent.doEvent();
    if (DrawLevel::file == update)
        state->update();
    if (DrawLevel::update == update || systemRedraw)
        state->redraw();
    else if (DrawLevel::draw == update)
        state->draw();
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    return SDL_APP_CONTINUE;
}

/* This function runs once per frame, and is the heart of the program. */
// on windows, at least, this appears to need to wait a bit before reading the file after finding
// that the file time has changed -- set it to repeatedly call update for some number of times
// once per frame
SDL_AppResult SDL_AppIterate(void* appstate) {
    DebuggerState* debuggerState = (DebuggerState*) appstate;
    return debuggerState->checkForNewFiles();
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
//    if (font)
//        TTF_CloseFont(font);  // !!! is this necessary?
    TTF_Quit();
}
