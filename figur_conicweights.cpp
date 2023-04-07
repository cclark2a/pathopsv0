#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "src/core/SkGeometry.h"
#include <string>

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
    bool up[] = { false, true, false, true };
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

void figur_conicweights(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    canvas->save();
    canvas->clear(SK_ColorWHITE);
    canvas->scale(2, 2);
    SkPoint conic[] = { { 100, 100 }, { 150, 100 }, { 150, 150 } };
    float weights[] = { 0, .35f, sqrtf(2) / 2, 1, 10 };
    canvas->translate(-50, 0);
    for (unsigned index = 0; index < ARRAY_COUNT(weights); ++index) {
        SkPath path;
        path.moveTo(conic[0]);
        path.conicTo(conic[1], conic[2], weights[index]);
        drawPath(canvas, path);
        drawTangents(canvas, conic, ARRAY_COUNT(conic));
        labelTangents(canvas, conic, ARRAY_COUNT(conic), "", false);
        canvas->translate(160, 0);
    }
    canvas->restore();
}
