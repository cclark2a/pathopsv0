/*
  Copyright (C) 1997-2025 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely.
*/
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

extern OpDebugPicture debugPicture;

#define DRAW_SDL 1

bool OpDebugSkipBreak() {
    return true;
}

bool debugUseAlt = false;

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture* bitmapTexture = NULL;
static SDL_Texture* textTexture = NULL;
#if 1
static int* frameBuffer = NULL;
#endif
static TTF_Font *font = NULL;
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 1000;

extern unsigned char tiny_ttf[];
extern unsigned int tiny_ttf_len;
OpContext* context = nullptr;
extern void V0D_AddEdges(OpContext* context);
extern void V0D_AddTangents();
extern void DebugColorEdges();
extern void V0D_ClearScreen();

unsigned int checkFile(void* userdata, SDL_TimerID timerID, Uint32 interval) {
    return interval;
}

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
    SDL_Color color = { 0, 0, 0, SDL_ALPHA_OPAQUE };
    SDL_Surface *text;
    /* Create the window */
    if (!SDL_CreateWindowAndRenderer("Hello World", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Couldn't create window and renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
  #if 1
    frameBuffer = (int*) malloc(WINDOW_WIDTH * WINDOW_HEIGHT * sizeof(int));
  bitmapTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, WINDOW_WIDTH, WINDOW_HEIGHT);
  #endif

    if (!TTF_Init()) {
        SDL_Log("Couldn't initialise SDL_ttf: %s\n", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    /* Open the font */
    font = TTF_OpenFont("C:/Windows/Fonts/segoeui.ttf", 18.0f * 4);
    if (!font) {
        SDL_Log("Couldn't open font: %s\n", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    /* Create the text */
    text = TTF_RenderText_Blended(font, "Hello World!", 0, color);
    if (text) {
        textTexture = SDL_CreateTextureFromSurface(renderer, text);
        SDL_DestroySurface(text);
    }
    if (!textTexture) {
        SDL_Log("Couldn't create text: %s\n", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_AddTimer(100, checkFile, nullptr);
    return SDL_APP_CONTINUE;
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
{
    if (event->type == SDL_EVENT_KEY_DOWN ||
        event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    }
    return SDL_APP_CONTINUE;
}

  #if 1
void render(Uint64 aTicks)
{
  for (int i = 0, c = 0; i < WINDOW_HEIGHT; i++)
  {
    for (int j = 0; j < WINDOW_WIDTH; j++, c++)
    {
      frameBuffer[c] = (int)(i * i + j * j + aTicks) | 0xff000000;
    }
  }
}
#endif

time_t lastTime = 0;

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void *appstate)
{
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
        V0D_ClearScreen();
        DebugColorEdges();
        V0D_AddEdges(context);
        debugPicture.setDevice();
        V0D_AddTangents();
        lastTime = info.st_mtime;
    }
    int w = 0, h = 0;
    SDL_FRect dst;
    const float scale = 1.0f;

    /* Center the text and scale it up */
    SDL_GetRenderOutputSize(renderer, &w, &h);
    SDL_SetRenderScale(renderer, scale, scale);
    SDL_GetTextureSize(textTexture, &dst.w, &dst.h);
    dst.x = ((w / scale) - dst.w) / 2;
    dst.y = ((h / scale) - dst.h) / 2;
#if 1
    if (!frameBuffer || !textTexture)
        return SDL_APP_CONTINUE;
    render(SDL_GetTicks());
#endif

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
#if 1
  char* pix;
  int pitch;
  SDL_LockTexture(bitmapTexture, NULL, (void**)&pix, &pitch);
#if 0
  for (int i = 0, sp = 0, dp = 0; i < WINDOW_HEIGHT; i++, dp += WINDOW_WIDTH, sp += pitch)
    memcpy(pix + sp, frameBuffer + dp, WINDOW_WIDTH * 4);
#else
    extern void pentrek_draw(char*, int width, int height, int pitch);
    pentrek_draw(pix, WINDOW_WIDTH, WINDOW_HEIGHT, pitch);
#endif
  SDL_UnlockTexture(bitmapTexture);  
  SDL_RenderTexture(renderer, bitmapTexture, NULL, NULL);
#endif
    SDL_RenderTexture(renderer, textTexture, NULL, &dst);
    SDL_RenderPresent(renderer);

    return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
    if (font)
        TTF_CloseFont(font);
    TTF_Quit();
}

#include "include/shim/surface.h"
#include "include/core/path_builder.h"
#include "src/raster/raster_canvas.h"

using namespace pentrek;

extern void draw_circle(Canvas*, ShimContext*);

void pentrek_draw(char* bits, int width, int height, int scan) {
    auto shim = ShimContext::MakeRaster();
    auto pm = Pixmap::C32(width, height, (Premul32*) bits, scan);
    RasterCanvas canvas(pm);
#if 0
    draw_circle(&canvas, shim.get());
#else
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
#endif
}
