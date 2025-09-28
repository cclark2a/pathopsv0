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

#include "OpContext.h"
#include "debug/OpDebugPicture.h"

#define DRAW_SDL 1

bool OpDebugSkipBreak() {
    return true;
}

static TTF_Font* font = nullptr;
OpPoint lastMouse;
static bool dragging = false;
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 1000;
OpContext* context = nullptr;
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
    pentrek_draw(pix, WINDOW_WIDTH, WINDOW_HEIGHT, pitch);
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

SDL_AppResult Window::init(std::string n, OpVector offset) {
    name = n;
    SDL_Color color = { 0, 0, 0, SDL_ALPHA_OPAQUE };
    if (!SDL_CreateWindowAndRenderer(("V0 Debugger " + name).c_str(), WINDOW_WIDTH, WINDOW_HEIGHT, 
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN, &window, &renderer)) {
        OpDebugOut("Couldn't create window and renderer: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
    int x, y;
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

static Window picture;
static Window text;

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
    picture.debugPicture.window = &picture;
    text.debugPicture.window = &text;
    drawIDsOn = true;  // !!! hardcode for testing
    drawEdgesOn = true;  // !!! hardcode for testing
    drawWindingsOn = false;  // !!! hardcode for testing
    drawValuesOn = true;  // !!! hardcode for testing
    drawGridOn = false; // !!! hardcode for testing
    drawPointsOn = true;
    drawFillOn = false;
    SDL_AppResult result = text.init("text", { -100, -100 });
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise text window: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
     result = picture.init("picture", { 100, 100 } );
    if (SDL_APP_CONTINUE != result) {
        OpDebugOut("Couldn't initialise picture window: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_FAILURE;
    }
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
    SDL_Keymod mod = SDL_GetModState();
    int scale = 1;
    if (SDL_KMOD_SHIFT & mod)
        scale *= 2;
    if (SDL_KMOD_CTRL & mod)
        scale *= 4;
    if (SDL_KMOD_ALT & mod)
        scale *= 16;
    Window* focused = SDL_GetWindowID(picture.window) == event->window.windowID ? &picture :
            SDL_GetWindowID(text.window) == event->window.windowID ? &text : nullptr;
    switch (event->type) {
        case SDL_EVENT_MOUSE_WHEEL: {
            if (!focused)
                return SDL_APP_CONTINUE;
            SDL_MouseWheelEvent& wheel = event->wheel;
            focused->debugPicture.zoom(wheel.y * scale);
            OpDebugOut("zoom:" + STR(focused->debugPicture.zoomFactor)
                    + " wheel.y:" + STR(wheel.y)
                    + " scale:" + STR(scale) + "\n");
            return SDL_APP_CONTINUE;
        } 
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            dragging = true;
            SDL_GetMouseState(&lastMouse.x, &lastMouse.y);
            return SDL_APP_CONTINUE;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: {
            dragging = false;
            return SDL_APP_CONTINUE;
        }
        case SDL_EVENT_MOUSE_MOTION: {
            if (!dragging || !focused)
                return SDL_APP_CONTINUE;
            float x, y;
            SDL_GetMouseState( &x, &y );
            if (lastMouse == OpPoint(x, y))
                return SDL_APP_CONTINUE;
            OpVector mouseMove = OpPoint(x, y) - lastMouse;
            focused->debugPicture.move(mouseMove);
            lastMouse = {x, y};
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (focused == &picture) {
                constexpr float pan_factor = 1.f / 8;
                if (SDLK_LSHIFT == event->key.key)
                    return SDL_APP_CONTINUE;
                switch (event->key.key) {
                    case SDLK_LEFT:
                        picture.debugPicture.pan(OpVector(-pan_factor * scale, 0));
                        break;
                    case SDLK_UP:
                        picture.debugPicture.pan(OpVector(0, -pan_factor * scale));
                        break;
                    case SDLK_RIGHT:
                        picture.debugPicture.pan(OpVector(pan_factor * scale, 0));
                        break;
                    case SDLK_DOWN:
                        picture.debugPicture.pan(OpVector(0, pan_factor * scale));
                        break;
                    case SDLK_C:
                        drawCentersOn ^= true;
                        break;
                    case SDLK_D:
                        if (SDL_KMOD_CTRL & mod)
                            picture.debugPicture.dump();
                        else if (SDL_KMOD_SHIFT & mod)
                            picture.debugPicture.setDepth(--picture.debugPicture.depth);
                        else
                            picture.debugPicture.setDepth(++picture.debugPicture.depth);
                        break;
                    case SDLK_E:
                        drawEdgesOn ^= true;
                        break;
                    case SDLK_F:
                        drawFillOn ^= true;
                        break;
                    case SDLK_G:
                        if (drawGridOn && !drawGridLinear)
                            drawGridLinear = true;
                        else {
                            drawGridOn ^= true;
                            drawGridLinear = false;
                        }
                        break;
                    case SDLK_H:
                        drawHullsOn ^= true;
                        break;
                    case SDLK_I:
                        drawIDsOn ^= true;
                        break;
                    case SDLK_K:
                        drawControlsOn ^= true;
                        break;
                    case SDLK_P:
                        drawPointsOn ^= true;
                        break;
                    case SDLK_S:
                        drawSegmentsOn ^= true;
                        break;
                    case SDLK_T:
                        drawTangentsOn ^= true;
                        break;
                    case SDLK_W:
                        drawWindingsOn ^= true;
                        break;
                    case SDLK_V:
                        drawValuesOn ^= true;
                        break;
                    case SDLK_X:
                        drawHexOn ^= true;
                        break;
                    case SDLK_0:
                    case SDLK_1:
                    case SDLK_2:
                    case SDLK_3:
                    case SDLK_4:
                    case SDLK_5:
                    case SDLK_6:
                    case SDLK_7:
                    case SDLK_8:
                    case SDLK_9:
                        debugPrecision = event->key.key - SDLK_0;
                        break;
                    case SDLK_MINUS:
                        debugPrecision = -1;
                        break;
                    case SDLK_GRAVE:
                        if (SDL_KMOD_SHIFT & mod)
                            picture.debugPicture.tuneThreshold ^= true;
                        break;
                }
            }
        }
        picture.debugPicture.redraw();
        text.debugPicture.redraw();
    }
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    return SDL_APP_CONTINUE;
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
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
        delete context;
        context = fromFile(opFileName);
        debugGlobalContext = context;
        if (context) {
            picture.debugPicture.context = context;
            picture.debugPicture.screen = OpRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
            picture.debugPicture.bootStrap();
            text.debugPicture.context = context;
            text.debugPicture.screen = OpRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        }
        lastTime = info.st_mtime;
    }
    picture.draw();
    text.draw();
    return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
    if (font)
        TTF_CloseFont(font);
    TTF_Quit();
}

#include "include/shim/surface.h"
// #include "include/canvas/fonts.h"
#include "include/core/path_builder.h"
#include "src/raster/raster_canvas.h"

using namespace pentrek;

#if 0
rcp<Font> gDefaultFont;

void pentrek_init() {
    if (auto data = Data::File("C:/Windows/Fonts/segoeui.ttf"))
        gDefaultFont = Font::MakePortable(data);
}
#endif

void Window::pentrek_draw(char* bits, int width, int height, int scan) {
    auto shim = ShimContext::MakeRaster();
    auto pm = Pixmap::C32(width, height, (Premul32*) bits, scan);
    RasterCanvas canvas(pm);
#if 0
    Paint clrPaint;
    clrPaint.color({1, 1, 1, 1});
    canvas.drawIRect({0, 0, width, height}, clrPaint);
#endif
    int debugCount = 0;
    for (OpDebugPoly& poly : debugPicture.polys) {
        if (poly.segment && !drawSegmentsOn)
            continue;
        if (poly.edge && !drawEdgesOn)
            continue;
        if (poly.contour && !drawFillOn)
            continue;
        size_t index = 0;
        for (size_t count : poly.contours) {
            Span<Point> points((Point*) (&poly.device.front() + index), count);
            index += count;
            PathBuilder bu;
            bu.addPoly(points, false);
            auto path = bu.snapshot();
            Paint paint;
            auto component = [poly](int bit) { return ((poly.color >> bit) & 0xFF) / 255.f; };
            paint.color({ component(16), component(8), component(0), component(24) });
//            paint.color({ 1, 0, 0, .3 });
            paint.stroke(!!poly.thickness);
            if (poly.thickness)
                paint.width(poly.thickness * 2);
            canvas.drawPath(path, paint);
            ++debugCount;
        }
    }
#if 0
    auto font = gDefaultFont;
    font = font->makeAt({'wght', 600});
    TextRun trun = { font, 100, 0, 5 };
    auto gruns = font->shapeCString("Hello", {&trun, 1});
    Paint paint;
    canvas.translate(20, 120);
    for (const auto& grun : gruns) {
        canvas.drawGlyphs(grun.m_glyphs, grun.m_xpos, 0, *grun.m_font, grun.m_size, paint);
    }
#endif
}
