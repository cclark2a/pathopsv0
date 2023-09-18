#include "include/core/SkCanvas.h"
#include "include/core/SkColor.h"
#include "include/core/SkPaint.h"
#include "include/core/SkPath.h"
#include "include/core/SkPoint.h"
#include "include/core/SkRect.h"
#include "include/core/SkSurface.h"

#include "src/pathops/SkIntersections.h"
#include "src/pathops/SkPathOpsLine.h"

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

void figur_coniccircle(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setStyle(SkPaint::kStroke_Style);
    float sAngles[] = { 45, 70, 90, 45, 45, 0 };
    float angles[] = { 45, 70, 90, 110, 135, 160 };
    float trans[] = { 120, 120, 120, 150, 180, 300 };
    for (int index = 0; index < ARRAY_COUNT(angles); ++index) {
        paint.setColor(0xFF7f7f7f);
        canvas->drawCircle({ 100, 100 }, 50, paint);
        SkPath path;
        SkPoint a = { 50 * sinf(sAngles[index] / 180 * 3.1416f),
            -50 * cosf(sAngles[index] / 180 * 3.1416f) };
        SkPoint b = { 50 * sinf((sAngles[index] + angles[index]) / 180 * 3.1416f),
            -50 * cosf((sAngles[index] + angles[index]) / 180 * 3.1416f) };
        SkIntersections i;
        SkDLine l1, l2;
        SkPoint l1Pts[] = { { 100 + a.fX - a.fY, 100 + a.fY + a.fX }, { 100 + a.fX, 100 + a.fY } };
        l1.set(l1Pts);
        SkPoint l2Pts[] = { { 100 + b.fX + b.fY, 100 + b.fY - b.fX }, { 100 + b.fX, 10130 + b.fY } };
        l2.set(l2Pts);
        int result = i.intersectRay(l1, l2);
        path.moveTo(100 + a.fX, 100 + a.fY);
        path.lineTo(100, 100);
        path.lineTo(100 + b.fX, 100 + b.fY);

        canvas->drawPath(path, paint);
        canvas->drawArc({ 70, 70, 130, 130 }, 270 + sAngles[index], angles[index], false, paint);
        paint.setColor(SK_ColorBLACK);
        path.reset();
        SkPoint conicPts[3] = { { (float)l1Pts[1].fX, (float)l1Pts[1].fY },
            { (float)i.pt(0).fX, (float)i.pt(0).fY },
            { (float)l2Pts[1].fX, (float)l2Pts[1].fY } };
        path.moveTo(conicPts[0]);
        path.conicTo(conicPts[1], conicPts[2], cosf(angles[index] / 180 * 3.1416f / 2));
        drawPath(canvas, path);
        drawTangents(canvas, conicPts, 3);
        labelTangents(canvas, conicPts, 3, "", false);

        canvas->translate(trans[index], 0);
    }
}