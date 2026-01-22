// (c) 2025, Cary Clark cclark2@gmail.com
#ifndef OP_TEST_RASTER
#define OP_TEST_RASTER 1  // !!! shouldn't be needed, but intelliSense doesn't work without it
#endif
#include "CompareWindow.h"
#include "DebuggerState.h"
#include "OpContext.h"
#include "OpDebugRaster.h"
#include <sys/stat.h>
#include <SDL3/SDL.h>

// !!! eventually move this into debugging strings in curves or maybe debugger bits txt
const std::vector<std::string> drawCompareStrs {
    "(unknown)",
    "contours resolved",
    "contour input",
    "segment input",
    "segments resolved",
    "edges",
    "output"
};

std::string CompareLabel::labelAt(int index) {
    if (!window->debugRaster)
        return "(uninitialized)";
    const DebugRaster* raster = window->debugRaster;
    const OpDebugSamples& sample = raster->samples[index];
    std::string postfix;
    const OpContext* context = raster->context;
    PathOpsV0Lib::DebugImageWindingNames nameFunc 
            = context->debugContextCallbacks.debugImageWindingNamesFuncPtr;
    std::vector<std::string> names;
    if (nameFunc)
        names = (*nameFunc)();
    auto inputName = [&names, sample, raster](int index) {
        if (names.empty())
            return std::string("");
        size_t strIndex = 0;
        for (int testIndex = 0; testIndex < raster->samples.size(); ++testIndex) {
            const OpDebugSamples& test = raster->samples[testIndex];
            if (test.sampleType != sample.sampleType)
                continue;
            if (testIndex == index) {
                OP_ASSERT(strIndex < names.size());
                return " (" + names[strIndex] + ")";
            }
            ++strIndex;
        }
        return std::string("");
    };
    switch (sample.sampleType) {
        case SampleType::contourInput:
            index = 2;
            postfix = inputName(index);
            break;
        case SampleType::contourResolved:
            index = 1;
            break;
        case SampleType::segmentInput:
            index = 3;
            postfix = inputName(index);
            break;
        case SampleType::segmentResolved:
            index = 4;
            break;
        case SampleType::edges:
            index = 5;
            break;
        case SampleType::output:
            index = 6;
            break;
        default:
            index = 0;
            break;
    }
    return drawCompareStrs[index] + postfix;
}

int CompareLabel::size() const {
    const DebugRaster* raster = window->debugRaster;
    if (!raster)
        return 0;
    return (int) raster->samples.size();
}

CompareWindow::CompareWindow(DebuggerState* state)
    : DebuggerWindow(state, WheelTarget::none)
    , leftLabel(this)
    , rightLabel(this) {
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
    if (!debugRaster)
        return SDL_APP_CONTINUE;
    if (debugRaster->samples.size() <= std::max(leftLabel.lastIndex, rightLabel.lastIndex))
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
            (OpDebugBitmap& srcBits, OpRect& bitFocus, CompareHalf half, uint32_t filter) {
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
            auto to32 = [&srcLine, &srcHPos, filter]() {
                uint8_t c = 0xFF - srcLine[(int) srcHPos];
                uint32_t color = (0xFF << 24) | (c << 0) | (c << 8) | (c << 16);
                color |= filter;
                return color;
            };
            uint32_t srcColor = to32();
            for (int dH = dstH; dH < dstRight; ++dH) {
                OP_ASSERT(destPtr < pixels + rowWidth * dstBottom);
                *destPtr++ &= srcColor;
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
    auto clearPixels = [pixels, winWidth, winHeight, rowWidth]() {
        uint32_t* destLine = pixels;
        for (int dstV = 0; dstV < winHeight; ++dstV) {
            uint32_t* destPtr = destLine;
            for (int dH = 0; dH < winWidth; ++dH) {
                *destPtr++ = 0xFFFFFFFF;
            }
            destLine += rowWidth;
        }
    };
    clearPixels(); 
    drawHalf(debugRaster->samples[leftLabel.lastIndex].mask, leftFocus, CompareHalf::left, 
            0);
    if (!overlay)
        drawHalf(debugRaster->samples[rightLabel.lastIndex].mask, rightFocus, CompareHalf::right, 
                0);
    else {
        drawHalf(debugRaster->samples[leftLabel.lastIndex].mask, leftFocus, CompareHalf::right, 
                0xFF00FFFF);
        drawHalf(debugRaster->samples[rightLabel.lastIndex].mask, rightFocus, CompareHalf::right, 
                0xFFFFFF00);
    }
    SDL_UnlockTexture(polysTexture);  
    SDL_RenderTexture(renderer, polysTexture, nullptr, nullptr);
    drawText();
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

DrawLevel CompareWindow::hover(const DebuggerEvent* event) {
    DrawLevel result = DrawLevel::none;
    if (!debugRaster)
        return result;
    // find set of samples corresponding to mouse position
    int windowWidth, windowHeight;
    if (!SDL_GetWindowSize(window, &windowWidth, &windowHeight)) {
        OpDebugOut("failed to get window size: " + std::string(SDL_GetError()) + "\n");
        return result;
    }
    int index = event->mouse.x < windowWidth / 2 ? leftLabel.lastIndex : rightLabel.lastIndex;
    if (index < 0 || index >= (int) debugRaster->samples.size())
        return result;
    const OpDebugSamples& sample = debugRaster->samples[index];
    OpRect area;
    if (event->mouse.x < windowWidth / 2) {
        area = OpRect(xOffset, yOffset, 0, 0);
        area.right = area.left + leftFocus.width() * scale;
        area.bottom = area.top + leftFocus.height() * scale;
    } else {
        area = OpRect(xOffset + (leftFocus.width() + margin) * scale, yOffset, 0, 0);
        area.right = area.left + rightFocus.width() * scale;
        area.bottom = area.top + rightFocus.height() * scale; 
    }
    activeRow = -1;
    // active row needs to be between 0 and pixel height * raster sub samples
    // needs to be different once scrolling / zooming is supported, but not now
    if (event->mouse.y >= area.top) {
        int rowLimit = debugRaster->bitHeight * debugRaster->subSamples;
        if (event->mouse.y > area.bottom)
            activeRow = rowLimit - 1;
        else {
            activeRow = (int) ((event->mouse.y - area.top) * rowLimit / (area.bottom - area.top));
            activeRow = std::max(0, std::min(activeRow, rowLimit - 1));
        }
    }
    if (-1 == activeRow)
        return result;
    OP_ASSERT(0 <= activeRow && activeRow < sample.sampleSet.size());
    const RasterSamples& samples = sample.sampleSet[activeRow];
    activeSample = nullptr;
    float locX = (event->mouse.x - area.left) / scale;
    for (const RasterSample& test : samples) {
        if (test.x <= locX)
            activeSample = &test;
    }
    // !!! use x position to find RasterSample to the left
    if (activeSample) 
        return DrawLevel::update;
    return result;
}

DrawLevel CompareWindow::event(const DebuggerEvent& event) {
    if (DrawLevel common = debuggerState->eventCommon(event); DrawLevel::none != common)
        return common;
    if (MouseAction::move == event.mouseAction)
        return hover(&event);
    return DrawLevel::none;
}

bool CompareWindow::readBits() {
    if (debuggerState->dumps.empty())
        return false;
    DebuggerDump& lastDump = debuggerState->dumps.back();
    if (!lastDump.context->debugDescription.ends_with("resolved"))
        return false;
    if (lastDump.context->callbacks.empty())
        return false;
    OpContext* context = lastDump.context;
    delete debugRaster;
    debugRaster = new DebugRaster(context);
    if (!debugRaster->playback(BitsFile))
        return false;
    leftFocus = { 0, 0, (float) debugRaster->bitWidth, (float) debugRaster->bitHeight };
    rightFocus = { 0, 0, (float) debugRaster->bitWidth, (float) debugRaster->bitHeight };
    return true;
}

void CompareWindow::update() {
    clearWindow();
    if (!readBits())
        return;
    OpPoint localLocation(10, 10);
    TTF_Font* detailFont = debuggerState->textWindow.detailFont;
    addText(leftLabel.label(), localLocation, debugBlack, detailFont);
    localLocation.x += (rightFocus.width() + margin) * scale;
    addText(rightLabel.label(), localLocation, debugBlack, detailFont);
    if (activeSample) {
        OpPoint pt { activeSample->x, (float) activeRow / debugRaster->subSamples };
        double srcX = (pt.x - debugRaster->offsetX) / debugRaster->scale;
        double srcY = (pt.y - debugRaster->offsetY) / debugRaster->scale;
        std::string s;
        if (activeSample->contour)
            s = "contour id:" + STR(activeSample->contour->id) + " curve:" 
                    + STR(activeSample->curveIndex);
        else if (activeSample->segment)
            s = "sample id:" + STR(activeSample->segment->id);
        else if (activeSample->edge)
            s = "edge id:" + STR(activeSample->edge->id);
        if (activeSample->curveDown)
            s += " curveDown";
        if (activeSample->visible)
            s += " visible";
        s += " x:" + STR(srcX) + " (" + STR(pt.x) + ")";
        s += " y:" + STR(srcY) + " (" + STR(((float) activeRow / debugRaster->subSamples)) + ")";
        addText(s, { 10, 30}, debugBlack, detailFont);
    }
}
