// (c) 2025, Cary Clark cclark2@gmail.com
#include "CompareWindow.h"
#include "DebuggerState.h"
#define TEST_RASTER 1
#include "OpDebugRaster.h"
#include <sys/stat.h>
#include <SDL3/SDL.h>

// !!! eventually move this into debugging strings in curves or maybe debugger bits txt
const std::vector<std::string> drawCompareStrs {
    "contour resolved",
    "contour input (left)",
    "contour input (right)",
    "segment input (left)",
    "segment input (right)",
    "segment resolved",
    "edge output"
};

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

// assume unit window is 2x wide (two side-by-side square bitmaps)
// scale bitmaps to fit
SDL_AppResult CompareWindow::draw() {
    if (!buffer)
        return SDL_APP_CONTINUE;
    
    SDL_SetRenderDrawColor(renderer, 0xEE, 0xEE, 0xEE, 255);
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
    // !!! current texture does not use pixelScale so don't use it below for now
    constexpr bool usePixScale = false;
    int pixScale = usePixScale ? pixelScale : 1;
    // pixelScale is only the 2x to handle hi-res screens
    // this needs to additionally scale focus dimensions to window dimensions
    // src bitmap is (0, 0, 64, 64)
    // focus could be anywhere relative to that
    // window is (0, 0, 2000, 2000) (e.g. windowWidth * pixelScale) 
    // destination for src pair should be ~ 0, 0, 2000, 2000 - 2100, 0, 4100, 2000 -ish
    // here's why window for compare should have different scale/offset from picture (grrr)
    int bitWidth = debugRaster->bitWidth * 2 + margin;
    int bitHeight = debugRaster->bitHeight;
    int winWidth = windowWidth * pixScale;
    int winHeight = windowHeight * pixScale;
    float xScale = (float) winWidth / bitWidth;
    float yScale = (float) winHeight / bitHeight;
    scale = std::min(xScale, yScale);
    xOffset = xScale == scale ? 0 : (winWidth - bitWidth * yScale) / 2;
    yOffset = yScale == scale ? 0 : (winHeight - bitHeight * xScale) / 2;
    // 'screen' is where focus is in window pixel coordinates; only draw part that intersects
    auto drawHalf = [this, pixels, winWidth, winHeight, rowWidth]
            (OpDebugBitmap& srcBits, OpRect& bitFocus, CompareHalf half) {
        screen = { xOffset, yOffset, 
                xOffset + bitFocus.width() * scale, yOffset + bitFocus.height() * scale };
        screen.offset({ -bitFocus.left * scale, -bitFocus.top * scale });
        if (screen.bottom <= 0 || screen.top >= winHeight)
            return;
        if (CompareHalf::right == half)
                screen = screen.offset({ (bitFocus.width() + margin) * scale, 0 });
        int dstV = std::max(0, (int) screen.top);
        int dstBottom = std::min(winHeight, (int) screen.bottom);
        if (dstV >= dstBottom)
                return;
        int dstH = std::max(0, (int) screen.left);
        int dstRight = std::min(winWidth, (int) screen.right);
        if (dstH >= dstRight)
                return;
        uint32_t* destLine = pixels + dstV * rowWidth + dstH;
        int srcV = std::max(0, (int) bitFocus.top);
        int srcBottom = std::min(debugRaster->bitHeight, (int) bitFocus.bottom);
        OP_ASSERT(srcV < srcBottom);
        int srcH = std::max(0, (int) bitFocus.left);
        int srcRight = std::min(debugRaster->bitWidth, (int) bitFocus.right);
        OP_ASSERT(srcH < srcRight);
        uint8_t* srcLine = &srcBits.bits.front() + srcV * debugRaster->bitWidth;
        float srcVPos = srcV;
        float srcBump = 1.f / scale;
        OP_ASSERT(0 <= dstV);
        OP_ASSERT(dstBottom <= winHeight);
        for (; dstV < dstBottom; ++dstV) {
            uint32_t* destPtr = destLine;
            OP_ASSERT(pixels + rowWidth * dstV <= destPtr);
            float srcHPos = srcH;
            auto to32 = [&srcLine, &srcHPos]() {
                uint8_t c = 0xFF - srcLine[(int) srcHPos];
                uint32_t color = (0xFF << 24) | (c << 0) | (c << 8) | (c << 16);
                return color;
            };
            uint32_t srcColor = to32();
            for (int dH = dstH; dH < dstRight; ++dH) {
                OP_ASSERT(destPtr < pixels + rowWidth * dstBottom);
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
    drawHalf(debugRaster->samples[leftBits].mask, leftFocus, CompareHalf::left);
    drawHalf(debugRaster->samples[rightBits].mask, rightFocus, CompareHalf::right);
    SDL_UnlockTexture(polysTexture);  
    SDL_RenderTexture(renderer, polysTexture, nullptr, nullptr);
    drawText();
    SDL_RenderPresent(renderer);
#if 0
    OpPoint localLocation(10, 10);
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    addText(drawCompareStrs[leftBits], localLocation, debugBlack, detailFont);
    localLocation.x += (rightFocus.width() + margin) * scale;
    addText(drawCompareStrs[rightBits], localLocation, debugBlack, detailFont);
#endif
    return SDL_APP_CONTINUE;
}

// events SDL_WINDOWEVENT_FOCUS_GAINED and SDL_WINDOWEVENT_FOCUS_LOST track which of 
// picture window and text window is top most; send events to that window
DrawLevel CompareWindow::event(const DebuggerEvent& event) {
    if (DrawLevel common = debuggerState->eventCommon(event); DrawLevel::none != common)
        return common;
    return DrawLevel::none;
}

bool CompareWindow::readBits() {
    if (!debugRaster)
        debugRaster = new DebugRaster(debuggerState->context);
    if (!debugRaster->playback(BitsFile))
        return false;
    leftFocus = { 0, 0, (float) debugRaster->bitWidth, (float) debugRaster->bitHeight };
    rightFocus = { 0, 0, (float) debugRaster->bitWidth, (float) debugRaster->bitHeight };
    return true;
}

void CompareWindow::update() {
    clearWindow();
    OpPoint localLocation(10, 10);
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    addText(drawCompareStrs[leftBits], localLocation, debugBlack, detailFont);
    localLocation.x += (rightFocus.width() + margin) * scale;
    addText(drawCompareStrs[rightBits], localLocation, debugBlack, detailFont);
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
