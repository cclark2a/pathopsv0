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
extern void pentrek_draw(char*, int width, int height, int pitch);

OpDebugPicture debugPicture;

#define DRAW_SDL 1

bool OpDebugSkipBreak() {
    return true;
}

bool debugUseAlt = false;

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture* bitmapTexture = NULL;
static int* frameBuffer = NULL;
static TTF_Font *font = NULL;
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 1000;

OpContext* context = nullptr;

std::vector<NativeTextCache> nativeTextCache;

size_t native_addText(std::string str, uint32_t color) {
    auto found = std::find_if(nativeTextCache.begin(), nativeTextCache.end(), 
            [str, color](NativeTextCache& cache) {
            return str == cache.str && color == cache.color; } );
    if (nativeTextCache.end() != found)
        return found - nativeTextCache.begin();
    uint32_t c = color;
    SDL_Color sdlColor = { (c >> 16) & 0xFF, (c >> 8) & 0xFF, (c >> 0) & 0xFF, (c >> 24) & 0xFF };
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, str.c_str(), 0, sdlColor);
    if (!textSurface) {
        OpDebugOut(std::string("Couldn't create text: %s\n") + SDL_GetError());
        exit(1);
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_DestroySurface(textSurface);
    size_t cacheIndex = nativeTextCache.size();
    OpVector size;
    SDL_GetTextureSize(texture, &size.dx, &size.dy);
    nativeTextCache.push_back({str, size, texture, color});
    return cacheIndex;
}

const NativeTextCache& native_cache(size_t index) {
    OP_ASSERT(index < nativeTextCache.size());
    return nativeTextCache[index];
}

std::string native_debugDump(size_t index) {
    OP_ASSERT(index < nativeTextCache.size());
    return nativeTextCache[index].debugDump();
}

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
    SDL_Color color = { 0, 0, 0, SDL_ALPHA_OPAQUE };
    if (!SDL_CreateWindowAndRenderer("V0 Debugger", WINDOW_WIDTH, WINDOW_HEIGHT, 
            SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Couldn't create window and renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    frameBuffer = (int*) malloc(WINDOW_WIDTH * WINDOW_HEIGHT * sizeof(int));
    bitmapTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, 
            SDL_TEXTUREACCESS_STREAMING,  WINDOW_WIDTH, WINDOW_HEIGHT);
    if (!TTF_Init()) {
        SDL_Log("Couldn't initialise SDL_ttf: %s\n", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    font = TTF_OpenFont("C:/Windows/Fonts/segoeui.ttf", 14);
    if (!font) {
        SDL_Log("Couldn't open font: %s\n", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
    SDL_Keymod mod = SDL_GetModState();
    float scale = 1;
    if (SDL_KMOD_SHIFT & mod)
        scale *= 2;
    if (SDL_KMOD_CTRL & mod)
        scale *= 4;
    if (SDL_KMOD_ALT & mod)
        scale *= 16;
    if (event->type == SDL_EVENT_MOUSE_WHEEL) {
        SDL_MouseWheelEvent& wheel = event->wheel;
        debugPicture.zoom(wheel.y * scale);
        return SDL_APP_CONTINUE;
    }
    if (event->type == SDL_EVENT_KEY_DOWN) {
        switch (event->key.key) {
            case SDLK_LEFT:
                debugPicture.pan(OpVector(-1, 0) * scale);
                break;
            case SDLK_UP:
                debugPicture.pan(OpVector(0, -1) * scale);
                break;
            case SDLK_RIGHT:
                debugPicture.pan(OpVector(1, 0) * scale);
                break;
            case SDLK_DOWN:
                debugPicture.pan(OpVector(0, 1) * scale);
                break;
        }
    }
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    return SDL_APP_CONTINUE;
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void *appstate) {
    static time_t lastTime = 0;
    struct stat info;    
    const char* opFileName = "d:/gerrit/skia/out/Debug/obj/dmp2.txt";
    if (stat(opFileName, &info) == -1) {
        assert(0);
        return SDL_APP_FAILURE;
    }
    if (info.st_mtime != lastTime) {
        delete context;
        context = fromFile(opFileName);
        debugGlobalContext = context;
        debugPicture.screen = OpRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        debugPicture.bootStrap(context);
        lastTime = info.st_mtime;
    }
    if (!frameBuffer)
        return SDL_APP_CONTINUE;
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    char* pix;
    int pitch;
    SDL_LockTexture(bitmapTexture, NULL, (void**)&pix, &pitch);
    pentrek_draw(pix, WINDOW_WIDTH, WINDOW_HEIGHT, pitch);
    SDL_UnlockTexture(bitmapTexture);  
    SDL_RenderTexture(renderer, bitmapTexture, NULL, NULL);
    for (OpDebugText& text : debugPicture.texts) {
        OP_ASSERT(text.cacheIndex < nativeTextCache.size());
        NativeTextCache& cache = nativeTextCache[text.cacheIndex];
        SDL_Texture* texture = (SDL_Texture*) cache.texture;
        SDL_FRect dst { text.pt.x, text.pt.y, cache.size.dx, cache.size.dy };
        SDL_RenderTexture(renderer, texture, NULL, &dst);
    }
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void *appstate, SDL_AppResult result) {
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

void pentrek_draw(char* bits, int width, int height, int scan) {
    auto shim = ShimContext::MakeRaster();
    auto pm = Pixmap::C32(width, height, (Premul32*) bits, scan);
    RasterCanvas canvas(pm);
    int debugCount = 0;
    for (OpDebugPoly& poly : debugPicture.polys) {
        size_t index = 0;
        for (size_t count : poly.contours) {
            Span<Point> points((Point*) (&poly.device.front() + index), count);
            index += count;
            PathBuilder bu;
            bu.addPoly(points, false);
            auto path = bu.snapshot();
            Paint paint;
            uint32_t color = (poly.color & 0xFF00FF00) | ((poly.color & 0xFF) << 16) 
                    | ((poly.color >> 16) & 0xFF);
            paint.color(Color::FromColor32(color));
            paint.stroke(!!poly.thickness);
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
