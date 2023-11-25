/*
* Copyright 2017 Google Inc.
*
* Use of this source code is governed by a BSD-style license that can be
* found in the LICENSE file.
*/
// (c) 2023, Cary Clark cclark2@gmail.com

#include "HelloWorld.h"

#include "include/core/SkCanvas.h"
#include "include/core/SkColor.h"
#include "include/core/SkFont.h"
#include "include/core/SkFontTypes.h"
#include "include/core/SkGraphics.h"
#include "include/core/SkPaint.h"
#include "include/core/SkPoint.h"
#include "include/core/SkRect.h"
#include "include/core/SkShader.h"
#include "include/core/SkString.h"
#include "include/core/SkSurface.h"
#include "include/core/SkTileMode.h"
#include "include/effects/SkGradientShader.h"
//#include "tools/sk_app/DisplayParams.h"


#include "OpCurve.h"
#include "PathOps.h"

#include <string>

void OpTest(bool terminateEarly);
void OpQuadDraw(SkCanvas* canvas);
void OpConicDraw(SkCanvas* canvas);
void OpCubicDraw(SkCanvas* canvas);

#include "src/pathops/SkIntersections.h"
#include "src/pathops/SkPathOpsLine.h"

void figur_monotonicity(SkSurface* surface);

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
//        , fRotationAngle(0)
{
    SkGraphics::Init();

    fWindow = Window::CreateNativeWindow(platformData);
    fWindow->setRequestedDisplayParams(DisplayParams());

    // register callbacks
    fWindow->pushLayer(this);

    fWindow->attach(fBackendType);
}

HelloWorld::~HelloWorld() {
    fWindow->detach();
    delete fWindow;
}

void HelloWorld::updateTitle() {
    if (!fWindow) {
        return;
    }

    SkString title("Hello World ");
    if (Window::kRaster_BackendType == fBackendType) {
        title.append("Raster");
    } else {
#if defined(SK_GL)
        title.append("GL");
#elif defined(SK_VULKAN)
        title.append("Vulkan");
#elif defined(SK_DAWN)
        title.append("Dawn");
#else
        title.append("Unknown GPU backend");
#endif
    }

    fWindow->setTitle(title.c_str());
}

void HelloWorld::onBackendCreated() {
    this->updateTitle();
    fWindow->onResize(768, 256);
    fWindow->show();
    fWindow->inval();
}

void drawPath(SkCanvas* canvas, const SkPath& path) {
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setStyle(SkPaint::kStroke_Style);
    canvas->drawPath(path, paint);
}

static void drawTangents(SkCanvas* canvas, const SkPoint* pts, int count) {
    SkPath tangents;
    tangents.moveTo(pts[0]);
    for (int x = 1; x < count; ++x)
        tangents.lineTo(pts[x]);
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setStyle(SkPaint::kStroke_Style);
    paint.setColor(0xFF7f7f7f);
    canvas->drawPath(tangents, paint);
}

static std::string ptToString(const SkPoint& pt) {
    return "(" + std::to_string((int)pt.fX) + ", " + std::to_string((int)pt.fY) + ")";
}

static void labelTangents(SkCanvas* canvas, const SkPoint* pts, int count, const char* indexAsString,
    bool drawLabel) {
    sk_sp<SkTypeface> typeface = SkTypeface::MakeDefault();
    SkFont font(typeface, 14);
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setColor(0xFF000000);
    for (int x = 0; x < count; ++x) {
        const SkPoint& pt = pts[x];
        paint.setStyle(SkPaint::kStroke_Style);
        canvas->drawCircle(pt, 2, paint);
        if (1 != x || !drawLabel)
            continue;
        paint.setStyle(SkPaint::kFill_Style);
        std::string ptStr = indexAsString ? indexAsString : ptToString(pt);
        SkRect bounds;
        SkScalar width = font.measureText(ptStr.c_str(), ptStr.length(), SkTextEncoding::kUTF8, &bounds);
        SkPoint labelPt = pt;
        switch (x) {
        case 0: labelPt.fX -= width * 5 / 4; labelPt.fY += bounds.height() / 2; break;
        case 1: labelPt.fX += width / 2; /* labelPt.fY -= bounds.height() / 2; */ break;
        case 2: labelPt.fX -= width / 2; labelPt.fY += bounds.height(); break;
        case 3: labelPt.fX -= width * 5 / 4; labelPt.fY += bounds.height() / 2; break;
        }
        canvas->drawSimpleText(ptStr.c_str(), ptStr.length(), SkTextEncoding::kUTF8,
            labelPt.fX, labelPt.fY, font, paint);
    }
}

void figur_coniccircle(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setStyle(SkPaint::kStroke_Style);
    float sAngles[] = { 45, 70, 90, 45, 45, 0 };
    float angles[] = { 45, 70, 90, 110, 135, 160 };
    float trans[] = {120, 120, 120, 150, 180, 300 };
    for (unsigned index = 0; index < ARRAY_COUNT(angles); ++index) {
        paint.setColor(0xFF7f7f7f);
        canvas->drawCircle({100, 100}, 50, paint);
        SkPath path;
        SkPoint a = { 50 * sinf(sAngles[index] / 180 * 3.1416f),
                     -50 * cosf(sAngles[index] / 180 * 3.1416f) };
        SkPoint b = { 50 * sinf((sAngles[index] + angles[index]) / 180 * 3.1416f),
                     -50 * cosf((sAngles[index] + angles[index]) / 180 * 3.1416f) };
        SkIntersections i;
        SkDLine l1, l2;
        SkPoint l1Pts[] = { { 100 + a.fX - a.fY, 100 + a.fY + a.fX }, { 100 + a.fX, 100 + a.fY } };
        l1.set(l1Pts);
        SkPoint l2Pts[] = { { 100 + b.fX + b.fY, 100 + b.fY - b.fX }, { 100 + b.fX, 100 + b.fY } };
        l2.set(l2Pts);
        (void) i.intersectRay(l1, l2);
        path.moveTo(100 + a.fX, 100 + a.fY);
        path.lineTo(100, 100);
        path.lineTo(100 + b.fX, 100 + b.fY);

        canvas->drawPath(path, paint);
        canvas->drawArc({70, 70, 130, 130}, 270 + sAngles[index], angles[index], false, paint);
        paint.setColor(SK_ColorBLACK);
        path.reset();
        SkPoint conicPts[3] = {{ (float) l1Pts[1].fX, (float)l1Pts[1].fY },
                               { (float) i.pt(0).fX, (float) i.pt(0).fY },
                               { (float) l2Pts[1].fX, (float)l2Pts[1].fY }};
        path.moveTo(conicPts[0]);
        path.conicTo(conicPts[1], conicPts[2], cosf(angles[index] / 180 * 3.1416f / 2));
        drawPath(canvas, path);
        drawTangents(canvas, conicPts, 3);
        labelTangents(canvas, conicPts, 3, "", false);

        canvas->translate(trans[index], 0);
    }
}

#if 01

extern void cubics44dDraw(SkCanvas* canvas);

void HelloWorld::onPaint(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    canvas->save();
    canvas->clear(SK_ColorWHITE);
#if 0
    canvas->scale(100, 100);
    cubics44dDraw(canvas);
    canvas->restore();
    return;
#elif OP_DEBUG_IMAGE
    if (GENERATE_COLOR_FILES) {
        OpDebugGenerateColorFiles();
        return;
    }
#endif
    OpTest(nullptr != surface); // trickery to avoid compiler warning
#if 0
    canvas->scale(75, 75);
    OpQuadDraw(canvas);
    canvas->translate(5, 0);
    OpConicDraw(canvas);
    canvas->translate(5, 0);
    OpCubicDraw(canvas);
#if 0
    canvas->translate(5, 0);
    SkConicDraw(canvas);
#endif
#elif 1

    canvas->scale(2, 2);
    SkPath path, path2;
#if 0
    SkPoint pts[] = { { 100, 100 }, { 150, 50 }, { 200, 200 }, { 400, 100 } };
    path.moveTo(pts[0]);
    path.cubicTo(pts[1], pts[2], pts[3]);
    path2 = path;
#else
    SkRect r1 = { 1, 2, 5, 6 };
    SkRect r2 = { 3, 4, 7, 8 };
    path.addRect(r1);
    path2.addRect(r2);
#endif
	OpInPath op1(&path);
	OpInPath op2(&path2);
	OpOutPath opOut(&path);
    PathOps(op1, op2, OpOperator::Intersect, opOut);
    drawPath(canvas, path);
#else
    // figur_monotonicity(surface);
    // figur_conicweights(surface);
    figur_coniccircle(surface);
#endif
    canvas->restore();
}
#else

void HelloWorld::onPaint(SkSurface* surface) {
    auto canvas = surface->getCanvas();

    // Clear background
    canvas->clear(SK_ColorWHITE);

    SkPaint paint;
    paint.setColor(SK_ColorRED);

    // Draw a rectangle with red paint
    SkRect rect = SkRect::MakeXYWH(10, 10, 128, 128);
    canvas->drawRect(rect, paint);

    // Set up a linear gradient and draw a circle
    {
        SkPoint linearPoints[] = { { 0, 0 }, { 300, 300 } };
        SkColor linearColors[] = { SK_ColorGREEN, SK_ColorBLACK };
        paint.setShader(SkGradientShader::MakeLinear(linearPoints, linearColors, nullptr, 2,
                                                     SkTileMode::kMirror));
        paint.setAntiAlias(true);

        canvas->drawCircle(200, 200, 64, paint);

        // Detach shader
        paint.setShader(nullptr);
    }

    // Draw a message with a nice black paint
    SkFont font;
    font.setSubpixel(true);
    font.setSize(20);
    paint.setColor(SK_ColorBLACK);

    canvas->save();
    static const char message[] = "Hello World ";

    // Translate and rotate
    canvas->translate(300, 300);
    fRotationAngle += 0.2f;
    if (fRotationAngle > 360) {
        fRotationAngle -= 360;
    }
    canvas->rotate(fRotationAngle);

    // Draw the text
    canvas->drawSimpleText(message, strlen(message), SkTextEncoding::kUTF8, 0, 0, font, paint);

    canvas->restore();
}
#endif

void HelloWorld::onIdle() {
    // Just re-paint continuously
    // fWindow->inval();
}

bool HelloWorld::onChar(SkUnichar c, skui::ModifierKey modifiers) {
    if (' ' == c) {
        if (Window::kRaster_BackendType == fBackendType) {
#if defined(SK_GL)
            fBackendType = Window::kNativeGL_BackendType;
#elif defined(SK_VULKAN)
            fBackendType = Window::kVulkan_BackendType;
#elif defined(SK_DAWN)
            fBackendType = Window::kDawn_BackendType;
#else
            SkDebugf("No GPU backend configured\n");
            return true;
#endif
        } else {
            fBackendType = Window::kRaster_BackendType;
        }
        fWindow->detach();
        fWindow->attach(fBackendType);
    }
    return true;
}

bool HelloWorld::onMouse(int x, int y, skui::InputState state, 
        skui::ModifierKey modifiers) {

    return true;
}
