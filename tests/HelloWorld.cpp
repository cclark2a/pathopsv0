/*
* Copyright 2017 Google Inc.
*
* Use of this source code is governed by a BSD-style license that can be
* found in the LICENSE file.
*/
// (c) 2023, Cary Clark cclark2@gmail.com

#define OP_INTERACTIVE 0

#include "HelloWorld.h"
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkGraphics.h"
#include "include/core/SkPath.h"
#include "include/core/SkSurface.h"
#include "PathOps.h"
#include "OpContour.h"
#include "OpMath.h"

#if !OP_INTERACTIVE
void OpTest(bool terminateEarly);
#endif

using namespace sk_app;

Application* Application::Create(int argc, char** argv, void* platformData) {
    return new HelloWorld(argc, argv, platformData);
}

struct PointIndex {
    int index;
    int close;
};

#if 0
struct PointsVerbs {
    // !!! add way to visualize edge runs
    SkPath makePath() {
        return SkPath::Make(&points.front(), points.size(),
            &verbs.front(), verbs.size(), &weights.front(), weights.size(), fillType);
    }
    
    void set(OpInPath& inPath) {
        SkPath* path = (SkPath*) inPath.externalReference;
        int count = path->countPoints();
        points.resize(count);
        path->getPoints(&points.front(), count);
        count = path->countVerbs();
        verbs.resize(count);
        path->getVerbs(&verbs.front(), count);
        curves = 0;
        fillType = path->getFillType();
        SkPath::RawIter iter(*path);
        SkPath::Verb verb;
        do {
            SkPoint pts[4];
            verb = iter.next(pts);
            curves += SkPath::kQuad_Verb <= verb && verb <= SkPath::kCubic_Verb;
            if (SkPath::kConic_Verb == verb)
                weights.push_back(iter.conicWeight());
        } while (verb != SkPath::kDone_Verb);
    }

    std::vector<SkPoint> points;
    std::vector<uint8_t> verbs;
    std::vector<SkScalar> weights;
    int curves;
    SkPathFillType fillType;
};

PointsVerbs leftPath;
PointsVerbs rightPath;
PointsVerbs* activePtV;
#endif

PointIndex activeIndex;
int activeFocus;
int activeLeft;
int activeRight;

	namespace skiatest {
	struct Reporter;
	}

void resetPaths() {
    OP_ASSERT(0); // !!! incomplete
    activeIndex = { -1, -1 };
    activeFocus = -1;
    activeLeft = 0;
    activeRight = 0;
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
    OpContours* fileContours = fromFile();
    std::swap(fileContours, debugGlobalContours);
    // preserve original contour data so it can be edited/restored later (incomplete)
    resetPaths();
    OpDebugImage::init();
    resetFocus();
    showEdges();
    showIDs();
#endif
    extern void testNewInterface();
    extern void testFrame();
    if ((0))
        testNewInterface();
    if ((1))
        testFrame();
//    extern void cubicOp114asQuad(skiatest::Reporter* reporter, const char* filename);
 //   cubicOp114asQuad(nullptr, "");
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
#if OP_DEBUG_IMAGE
    if (GENERATE_COLOR_FILES) {
        OpDebugGenerateColorFiles();
        return;
    }
#endif
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

#if 0
PointIndex pointIndex(PointsVerbs& ptVerbs, int curve, int focus) {
    PointIndex result {-1, -1};
    size_t vIndex = 0;
    size_t ptIndex = 0;
    size_t firstInContour = 0;
    int cIndex = curve;
    OP_ASSERT(focus > 0 && focus <= 4);
    while (vIndex < ptVerbs.verbs.size() && cIndex >= 0) {
        SkPath::Verb v = (SkPath::Verb) ptVerbs.verbs[vIndex++];
        if (SkPath::kMove_Verb == v || SkPath::kLine_Verb == v) {
            ++ptIndex;
            continue;
        }
        if (SkPath::kClose_Verb == v || SkPath::kDone_Verb == v) {
            if (firstInContour == (size_t) result.index && 1 < ptIndex) {
                size_t closePtIndex = ptIndex - 1;
                if (ptVerbs.points[firstInContour] == ptVerbs.points[closePtIndex])
                    result.close = closePtIndex;
                break;
            }
            if (SkPath::kDone_Verb == v)
                break;
        }
        if (SkPath::kClose_Verb == v) {
            firstInContour = ptIndex;
            continue;
        }
        int ptCount = 0;
        switch (v) {
            case SkPath::kQuad_Verb:
                ptCount = 3;
            break;
            case SkPath::kConic_Verb:
                ptCount = 3;
            break;
            case SkPath::kCubic_Verb:
                ptCount = 4;
            break;
            default:
                OP_ASSERT(0);
        }
       if (0 == cIndex) {
           if (focus > ptCount)
               return result;
           int offset = focus - 2;  // first point is 1, so -1 is last point from prior 
           OP_ASSERT((int) ptIndex >= offset);
           result.index = ptIndex + offset;
       }
       --cIndex;
       ptIndex += ptCount - 1;
       OP_ASSERT(ptIndex < ptVerbs.points.size());
    }
    return result;
}
#endif

int debugBreakDepth;  // !!! this needs to be moved into global contour

void adjustCCDepth(int delta) {
    int breakDepth = debugBreakDepth + delta;
    if (breakDepth < 1)
        return;
    debugBreakDepth = breakDepth;
}

bool HelloWorld::onChar(SkUnichar c, skui::ModifierKey modifiers) {
#if OP_DEBUG_IMAGE
    switch (c) {
    case 'b' : toggleBounds(); break;
    case 'c' : toggleControls(); break;
    case 'C' : toggleControlLines(); break;
    case 'd' : adjustCCDepth(+1); break;
    case 'D' : adjustCCDepth(-1); break;
    case 'e' : toggleEdges(); break;
    case 'E' : toggleEndToEnd(); break;
    case 'f' : toggleFill(); break;
    case 'g' : toggleGrid(); break;
    case 'G' : toggleGuides(); break;
    case 'h' : toggleHex(); break;
    case 'H' : toggleHulls(); break;
    case 'i' : toggleIDs(); break;
    case 'I' : toggleIntersections(); break;
    case 'n' : toggleNormals(); break;
    case 'N' : toggleIn(); break;
    case 'o' : togglePathsOut(); break;
    case 'O' : toggleOperands(); break;
    case 'p' : togglePoints(); break;
    case 'r' : resetPaths(); break;  // !!! add way to (probably toggle) original path and current edit
    case 's' : toggleSegments(); break;
    case 't' : toggleTangents(); break;
    case 'v' : toggleValues(); break;
    case 'w' : toggleWindings(); break;
    case '0' : 
        activeFocus = 0; 
//        activePtV = nullptr; 
        activeIndex = { -1, -1 };
        break;
    default:
        if ('1' <= c && c <= '8') {
            activeFocus = c <= '4' ? c - '0' : c - '4';
#if 0
            activePtV = c <= '4' ? &leftPath : &rightPath;
            int activeCurve = c <= '4' ? activeLeft : activeRight;
            activeIndex = pointIndex(*activePtV, activeCurve, activeFocus); 
#endif
        }
        break;
    }
#endif
    return true;
}

bool HelloWorld::onKey(skui::Key, skui::InputState, skui::ModifierKey) {
    return true;
}

// change indicated point. If this point is the first in contour, change last also if same
#if 0
void setPoint(PointsVerbs& ptVerbs, PointIndex pi, SkPoint pt) {
    OP_ASSERT(0 <= pi.index && pi.index < (int) ptVerbs.points.size());
    OP_ASSERT(-1 == pi.close || (0 < pi.close && pi.close < (int) ptVerbs.points.size()));
    SkPoint oldValue = ptVerbs.points[pi.index];
    OP_ASSERT(-1 == pi.close || oldValue == ptVerbs.points[pi.close]);
    ptVerbs.points[pi.index] = pt;
    if (0 < pi.close)
        ptVerbs.points[pi.close] = pt;
}
#endif

// associates 1:4 with first curve; 5:8 with second curve
void movePoint(float x, float y) {
    switch (activeFocus) {
        case 1: break;
        case 2: break;
        case 3: break;
        case 4: break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8: break;
    }
    // !!! start here
    // edit skia path
    OP_DEBUG_CODE(int ptIndex = activeFocus % 3);
    OP_ASSERT(ptIndex);  // !!! suppress warning; incomplete
    // regenerate segments, edges, curve/curve data (esp. edge runs, exiting when some depth is reached)
}

bool HelloWorld::onMouse(int x, int y, skui::InputState state, skui::ModifierKey modifiers) {
#if OP_DEBUG_IMAGE
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
        if (OpMax != lastX) {
            int deltaX = x - lastX;
            int deltaY = y - lastY;
           if (deltaX > 0)
               activeFocus <= 0 ? l(.02f) : movePoint(-.02f, 0);
           else if (deltaX < 0)
               activeFocus <= 0 ? r(.02f) : movePoint(.02f, 0);
           if (deltaY > 0)
               activeFocus <= 0 ? u(.02f) : movePoint(0, -.02f);
           else if (deltaY < 0)
               activeFocus <= 0 ? d(.02f) : movePoint(.02f, .02f);
        }
        lastX = x;
        lastY = y;
    }
#endif
    return true;
}

#if 0  // !!! interface changes across different versions of Skia ...
bool HelloWorld::onMouseWheel(float delta, skui::ModifierKey modifiers) {
#if OP_DEBUG_IMAGE
    if (delta > 0)
        i(1);
    else if (delta < 0)
        oo(1);
#endif
    return true;
}
#endif
