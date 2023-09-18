
#include "src/pathops/SkPathOpsCubic.h"
#include "src/pathops/SkPathOpsQuad.h"
#include "src/core/SkGeometry.h"
#include <string>

static void drawPath(SkCanvas* canvas, const SkPath& path) {
    SkPaint paint;
    paint.setAntiAlias(true);
    paint.setStyle(SkPaint::kStroke_Style);
    canvas->drawPath(path, paint);
}

static void drawTangents(SkCanvas* canvas, const SkPoint* pts, int count) {
    SkPath tangents;
    tangents.moveTo(pts[0]);
#if 1
    for (int x = 1; x < count; ++x)
        tangents.lineTo(pts[x]); 
#else	// when polybounds are drawn
    for (int x = 1; x < count; ++x) {
        int index = x;
        if (0 && 4 == count && (x == 2 || x == 3))
            index = x ^ 1;
        tangents.lineTo(pts[index]);
    }
    tangents.close();
#endif
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
        if (!drawLabel)
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

void figur_monotonicity(SkSurface* surface) {
    auto canvas = surface->getCanvas();
    canvas->save();
    canvas->clear(SK_ColorWHITE);
    canvas->scale(2, 2);
    SkPoint monoline[] = { { 100, 100 }, { 130, 200 } };
    SkPoint monoquad[] = { { 100, 100 }, { 150, 150 }, { 150, 200 } };
    SkPoint monocubic[] = { { 100, 100 }, { 130, 130 }, { 150, 170 }, { 150, 200 } };
    SkPoint notmonoquad[] = { { 100, 100 }, { 250, 200 }, { 150, 200 } };
    SkPoint notmonocubic[] = { { 100, 100 }, { 120, 170 }, { 150, 130 }, { 150, 200 } };
    const SkPoint* ptSets[] = { monoline, monoquad, monocubic, notmonoquad, notmonocubic };
    int ptsSetLengths[] = { ARRAY_COUNT(monoline), ARRAY_COUNT(monoquad), ARRAY_COUNT(monocubic),
    ARRAY_COUNT(notmonoquad), ARRAY_COUNT(notmonocubic) };
    int extremaCount;
    int inflectionCount;
    double dts[4];
    SkDQuad quad;
    SkDCubic cubic;
    int exes;
    for (unsigned index = 0; index < 5; ++index) {
        SkPath path;
        path.moveTo(ptSets[index][0]);
        switch (ptsSetLengths[index]) {
        case 2:
            path.lineTo(ptSets[index][1]);
            break;
        case 3:
            path.quadTo(ptSets[index][1], ptSets[index][2]);
            quad.set(ptSets[index]);
            extremaCount = exes = SkDQuad::FindExtrema(&quad.fPts[0].fX, dts);
            extremaCount += SkDQuad::FindExtrema(&quad.fPts[0].fY, &dts[exes]);
            for (int i2 = 0; i2 < extremaCount; ++i2) {
                SkPoint extrema = SkEvalQuadAt(ptSets[index], dts[i2]);
                labelTangents(canvas, &extrema, 1, "extrema", true);
            }
            break;
        case 4:
            path.cubicTo(ptSets[index][1], ptSets[index][2], ptSets[index][3]);
            cubic.set(ptSets[index]);
            extremaCount = exes = SkDCubic::FindExtrema(&cubic.fPts[0].fX, dts);
            extremaCount += SkDCubic::FindExtrema(&cubic.fPts[0].fY, &dts[exes]);
            for (int i2 = 0; i2 < extremaCount; ++i2) {
                if (dts[i2] > .95f)
                    continue;
                SkPoint loc;
                SkEvalCubicAt(ptSets[index], dts[i2], &loc, nullptr, nullptr);
                labelTangents(canvas, &loc, 1, "extrema", true);
            }
            inflectionCount = cubic.findInflections(dts);
            for (int i2 = 0; i2 < inflectionCount; ++i2) {
                SkPoint loc;
                SkEvalCubicAt(ptSets[index], dts[i2], &loc, nullptr, nullptr);
                labelTangents(canvas, &loc, 1, "inflection", true);
            }
            break;
        default:
            assert(0);
        }
        drawPath(canvas, path);
        drawTangents(canvas, ptSets[index], ptsSetLengths[index]);
        labelTangents(canvas, ptSets[index], ptsSetLengths[index], "", false);
        canvas->translate(100 + 130 * (index == 3), 0);
    }
    canvas->restore();
}