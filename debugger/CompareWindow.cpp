// (c) 2025, Cary Clark cclark2@gmail.com
#include "CompareWindow.h"
#include "DebuggerState.h"
#define TEST_RASTER 1
#include "OpDebugRaster.h"
#include <sys/stat.h>
#include <SDL3/SDL.h>

CompareWindow::CompareWindow(DebuggerState* state)
    : DebuggerWindow(state, WheelTarget::none) {
    pixelScale = 8;
    if (SDL_APP_CONTINUE != (state->error = init("compare", { -200, -200 })))
        OpDebugOut("Couldn't initialize compare window: " + std::string(SDL_GetError()) + "\n");
    else 
        SDL_HideWindow(window);
}

enum class CompareHalf {
    left,
    right
};

SDL_AppResult CompareWindow::draw() {
    if (!buffer)
        return SDL_APP_CONTINUE;
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    uint32_t* pixels;
    int pitch;
    SDL_LockTexture(polysTexture, nullptr, (void**)&pixels, &pitch);
    int rowWidth = pitch / sizeof(uint32_t);
    int windowWidth, windowHeight;
    if (!SDL_GetWindowSize(window, &windowWidth, &windowHeight)) {
        OpDebugOut("failed to get window size: " + std::string(SDL_GetError()) + "\n");
        return SDL_APP_CONTINUE;
    }
    // to support future zooming and scrolling, fix the scale and offset

    auto drawHalf = [this, pixels, windowWidth, windowHeight, rowWidth]
            (OpDebugBitmap& srcBits, CompareHalf half) {
        screen = { 0, 0, focus.width() * pixelScale, focus.height() * pixelScale };
        screen.offset({ -focus.left * pixelScale, -focus.top * pixelScale });
        if (screen.bottom <= 0 || screen.top >= windowHeight)
            return;
        if (CompareHalf::right == half)
                screen.offset({ focus.width() * pixelScale + margin, 0 });
        int dstV = std::max(0, (int) screen.top);
        int dstBottom = std::min(windowHeight, (int) screen.bottom);
        if (dstV >= dstBottom)
                return;
        int dstH = std::max(0, (int) screen.left);
        int dstRight = std::min(windowWidth, (int) screen.right);
        if (dstH >= dstRight)
                return;
        uint32_t* destLine = pixels + dstV * rowWidth;
        int srcV = std::max(0, (int) focus.top);
        int srcBottom = std::min(debugRaster->bitHeight, (int) focus.bottom);
        OP_ASSERT(srcV < srcBottom);
        int srcH = std::max(0, (int) focus.left);
        int srcRight = std::min(debugRaster->bitWidth, (int) focus.right);
        OP_ASSERT(srcH < srcRight);
        uint8_t* srcLine = &srcBits.bits.front() + srcV * debugRaster->bitWidth;
        float srcVPos = srcV;
        float srcBump = 1 / pixelScale;
        for (; dstV < dstBottom; ++dstV) {
            uint32_t* destPtr = destLine;
            float srcHPos = srcH;
            auto to32 = [&srcLine, &srcHPos]() {
                uint8_t c = srcLine[(int) srcHPos];
                uint32_t color = 0xFF | (c << 8) | (c << 16) | (c << 24);
                return color;
            };
            uint32_t srcColor = to32();
            for (int dstH = 0; dstH < windowWidth; ++dstH) {
                OP_ASSERT(pixels <= destPtr && destPtr < pixels + rowWidth * dstBottom);
                *destPtr++ = srcColor;
                float hPos = srcHPos;
                srcHPos += srcBump;
                if (floorf(srcHPos) > hPos)
                    srcColor = to32();
            }
            destLine += rowWidth;
            float vPos = floorf(srcVPos);
            srcVPos += srcBump;
            if (floorf(srcVPos) > vPos)
                srcLine += debugRaster->bitWidth;
        }
    };
    // !!! placeholder : need to be able to choose any sample set for left and right
    drawHalf(debugRaster->samples[0].mask, CompareHalf::left);
    drawHalf(debugRaster->samples[1].mask, CompareHalf::right);
    SDL_UnlockTexture(polysTexture);  
    SDL_RenderTexture(renderer, polysTexture, nullptr, nullptr);
    drawText();
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel CompareWindow::event(const DebuggerEvent& event) {
    OP_ASSERT(debuggerState->lastFocus);
    return debuggerState->lastFocus->event(event);
}

bool CompareWindow::readBits() {
    if (!debugRaster)
        debugRaster = new DebugRaster(debuggerState->context);
    if (!debugRaster->playback(BitsFile))
        return false;
    focus = { 0, 0, (float) debugRaster->bitWidth, (float) debugRaster->bitHeight };
    screen = { 0, 0, focus.right * pixelScale * 2 + margin, focus.bottom * pixelScale };
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
            return;
        }
    }

}
