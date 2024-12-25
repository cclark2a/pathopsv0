/*
* Copyright 2017 Google Inc.
*
* Use of this source code is governed by a BSD-style license that can be
* found in the LICENSE file.
*/
// (c) 2023, Cary Clark cclark2@gmail.com

#include "HelloWorld.h"
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkGraphics.h"
#include "include/core/SkPath.h"
#include "include/core/SkSurface.h"
#include "OpTestDrive.h"

#if OP_INTERACTIVE
// interfaces are placeholders (considerably out of date)
void readFromFile();
void opOnChar(char c);
void opOnMouse(int lastX, int lastY, int x, int y);
void opWheel(float delta);
#else
void OpTest(bool terminateEarly);
#endif

using namespace sk_app;

Application* Application::Create(int argc, char** argv, void* platformData) {
    return new HelloWorld(argc, argv, platformData);
}

HelloWorld::HelloWorld(int argc, char** argv, void* platformData)
#if defined(SK_GL)
        : fBackendType(Window::kNativeGL_BackendType)
#elif defined(SK_VULKAN)
        : fBackendType(Window::kVulkan_BackendType)
#elif defined(SK_DAWN)
        : fBackendType(Window::kDawn_BackendType)
#else
        : fBackendType(Window::kRaster_BackendType)
#endif
{
    SkGraphics::Init();
    fWindow = Window::CreateNativeWindow(platformData);
    fWindow->setRequestedDisplayParams(DisplayParams());
    // register callbacks
    fWindow->pushLayer(this);
    fWindow->attach(fBackendType);
#if OP_INTERACTIVE
	readFromFile();
#endif
}

HelloWorld::~HelloWorld() {
    fWindow->detach();
    delete fWindow;
}

void HelloWorld::updateTitle() {
    if (!fWindow)
        return;
    fWindow->setTitle("Hello World ");
}

void HelloWorld::onBackendCreated() {
    this->updateTitle();
    fWindow->onResize(768, 256);
    fWindow->show();
    fWindow->inval();
}

void HelloWorld::onPaint(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    canvas->save();
    canvas->clear(SK_ColorWHITE);
#if OP_INTERACTIVE
    SkBitmap& bitmap = bitmapRef();
    sk_sp<SkImage> image1 = bitmap.asImage();
    canvas->drawImage(image1, 0, 0);
#else
    OpTest(nullptr != canvas); // trickery to avoid compiler warning
#endif
    canvas->restore();
}

void HelloWorld::onIdle() {
    // Just re-paint continuously
#if OP_INTERACTIVE
    fWindow->inval();
#endif
}

int debugBreakDepth;  // !!! this needs to be moved into global contour

void adjustCCDepth(int delta) {
    int breakDepth = debugBreakDepth + delta;
    if (breakDepth < 1)
        return;
    debugBreakDepth = breakDepth;
}

bool HelloWorld::onChar(SkUnichar c, skui::ModifierKey modifiers) {
#if OP_INTERACTIVE
	opOnChar((char) c);
#endif
    return true;
}

bool HelloWorld::onKey(skui::Key, skui::InputState, skui::ModifierKey) {
    return true;
}

bool HelloWorld::onMouse(int x, int y, skui::InputState state, skui::ModifierKey modifiers) {
#if OP_INTERACTIVE
    static bool mouseDown = false;
    static int lastX = OpMax, lastY = OpMax;
    if (skui::InputState::kUp == state) {
        mouseDown = false;
        return true;
    }
    if (skui::InputState::kDown == state)  {
        mouseDown = true;
        return true;
    }
    OP_ASSERT(skui::InputState::kMove == state);
    if (mouseDown) {
        if (OpMax != lastX)
			opOnMouse(lastX, lastY, x, y);
        lastX = x;
        lastY = y;
    }
#endif
    return true;
}

#if 0  // !!! interface changes across different versions of Skia ...
bool HelloWorld::onMouseWheel(float delta, skui::ModifierKey modifiers) {
#if OP_INTERACTIVE
	opWheel(delta);
#endif
    return true;
}
#endif
