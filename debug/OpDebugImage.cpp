#include "OpDebug.h"
#if OP_DEBUG_IMAGE

#ifdef _MSC_VER
#pragma optimize( "", off )
#endif

#include <algorithm>
#include <functional>
#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkFont.h"
#include "include/core/SkImageInfo.h"
#include "include/core/SkPath.h"
#include "include/core/SkPaint.h"
#endif
#include "OpCurveCurve.h"
#include "OpEdge.h"
#include "OpJoiner.h"
#include "OpMath.h"
#include "OpSegment.h"
#include "OpTightBounds.h"
#include "OpWinder.h"
#include "PathOps.h"
#include "DebugOpsTypes.h"

SkBitmap bitmap;
SkFont labelFont(nullptr, 14, 1, 0);  // windows by default: "Segoe UI"

std::vector<OpDebugRay> lines;
std::vector<PathOpsV0Lib::ColorCurve> curves;
std::vector<OpPtT> ptTs;
int gridIntervals = 8;
int rasterIntervals = 64;
int limbsShown = 0;  // <= 0 is show all

#define OP_X(Thing) \
bool draw##Thing##On = false;
MASTER_LIST
EDGE_BOOL_LIST
ALIAS_LIST
CALLOUT_LIST
#undef OP_X

SkBitmap& bitmapRef() {
	return bitmap;
}

#define DRAW_IDS_ON(Thing) \
	do { \
		draw##Thing##On = true; \
		drawIDsOn = true; \
	} while (false)


struct OpDebugDefeatDelete {
#if OP_DEBUG
	OpDebugDefeatDelete() {
		save = debugGlobalContext->debugInPathOps;
		debugGlobalContext->debugInPathOps = false;
	}
	~OpDebugDefeatDelete() {
		debugGlobalContext->debugInPathOps = save;
	}

	bool save;
#endif
};

void OpDebugImage::addToPath(const OpCurve& curve, SkPath& path) {
	path.moveTo(curve.firstPt().x, curve.firstPt().y);
	if ((size_t) curve.c.type > curve.context().debugCallbacks.size())
		return;
#if !OP_TINY_SKIA
	PathOpsV0Lib::DebugAddToPath debugAddToPath = nullptr; // !!! curve.context().debugCallback(curve.c).addToPathFuncPtr;
	if (!debugAddToPath)
		return;
	(*debugAddToPath)(curve.c, path);
#endif
}

void OpDebugImage::init() {
	bitmap.allocPixels(SkImageInfo::MakeN32Premul(bitmapWH, bitmapWH));
	::clear();
	focusSegments();
}

#if 0
static size_t playbackBytes(const char** strPtr, uint8_t* data, size_t size) {
    const char* str = *strPtr;
    const char* s = str;
    uint8_t* d = data;
    size_t sz = size;
    auto error = [str](std::string expected, char ch) {
        OpDebugOut("expected " + expected + "; got " + std::string(&ch, 1) + " (" + STR(ch) + "): " 
                + std::string(str));
        return 0;
    };
    while (*s && sz) {
        uint8_t byte = 0;
        char ch = *s++;
        if (ch != '0') {
            if (0 == size)
                break;
            return error("'0'", ch);
        }
        ch = *s++;
        if (ch != 'x')
            return error("'x'", ch);
        for (int index = 0; index < 2; ++index) {
            byte <<= 4;
            ch = *s++;
            if ('0' <= ch && ch <= '9')
                byte |= ch - '0';
            else if ('a' <= ch && ch <= 'f')
                byte |= ch - 'a' + 10;
            else
                return error("hex digit", ch);
        }
        if (data)
            *d = byte;
        d++;
        ch = *s++;
        if (ch != ' ')
            return error("' '", ch);
        if (size)
            sz -= 1;
    }
    if (data)
        *strPtr = s;
    return d - data;
}

void OpDebugImage::playback(FILE* file) {
	if (!file)
		return;
	// required
	if (double debugZoom; fscanf(file, "debugZoom: %lg\n", &debugZoom) != 1) {
		OpDebugOut("reading debugZoom failed\n");
		fclose(file);
		return;
	} else 
        DebugOpSetZoom(debugZoom);
	if (double debugCenter[2];fscanf(file, "debugCenter: %lg, %lg\n", &debugCenter[0], &debugCenter[1]) != 2) {
		OpDebugOut("reading debugCenter failed\n");
		fclose(file);
		return;
	} else
	    DebugOpSetCenter(debugCenter[0], debugCenter[1]);
	DebugOpResetBounds();
	if (float textSize; fscanf(file, "textSize: %g\n", &textSize) != 1) {
		OpDebugOut("reading textSize failed\n");
		fclose(file);
		return;
	} else
	    labelFont.setSize(textSize);
	if (int intervals; fscanf(file, "gridIntervals: %d\n", &intervals) != 1) {
		OpDebugOut("reading gridIntervals failed\n");
		fclose(file);
		return;
	} else
	    gridIntervals = intervals;
	if (int pPrecision; fscanf(file, "debugPrecision: %d\n", &pPrecision) != 1) {
		OpDebugOut("reading debugPrecision failed\n");
		fclose(file);
		return;
	} else     
	    debugPrecision = pPrecision;
	if (int pSmall; fscanf(file, "debugSmall: %d\n", &pSmall) != 1) {
		OpDebugOut("reading debugSmall failed\n");
		fclose(file);
		return;
	} else
	    debugSmall = pSmall;
	if (int pEpsilon; fscanf(file, "debugEpsilon: %d\n", &pEpsilon) != 1) {
		OpDebugOut("reading debugEpsilon failed\n");
		fclose(file);
		return;
	} else
	    debugEpsilon = pEpsilon;
	// optional
    auto strMatch = [](const char* str, const char* match) {
        if (strlen(str) > strlen(match) && 0 == strncmp(match, str, strlen(match)))
            return str + strlen(match);
        else
            return (const char*) nullptr;
    };
	{
	#define OP_X(Thing) \
		draw##Thing##On = false;
		MASTER_LIST
		EDGE_BOOL_LIST
		ALIAS_LIST
		CALLOUT_LIST
	#undef OP_X
    	char str[255];
		while (fgets(str, sizeof(str), file)) {
	        auto noMatch = [file, str](std::string label) {
		        OpDebugOut("no match: " + label + "; " + str); 
		        fclose(file);
	        };
	#define OP_X(Thing) \
			if (strlen(str) - 1 == strlen(#Thing) && 0 == strncmp(#Thing, str, strlen(#Thing) - 1)) { \
				draw##Thing##On = true; \
	    		show##Thing(); \
			} else
		MASTER_LIST
		EDGE_BOOL_LIST
		ALIAS_LIST
		CALLOUT_LIST
	#undef OP_X
		if (const char* idEdgeStr = strMatch(str, "edge: ")) {
				int id = strtol(idEdgeStr, nullptr, 0);
                OpEdge* edge = findEdge(id);
				if (edge) {  // ok if recorded edge does not exist
					const char* drawStr = strstr(idEdgeStr, "draw: ");
                    if (!drawStr)
                        return noMatch("draw");
                    edge->debugDraw = strtol(drawStr + strlen("draw: "), nullptr, 0);
                    edge->debugOne = true;
					const char* colorStr = strstr(idEdgeStr, "color: ");
					if (!colorStr)
                        return noMatch("color");
                    edge->debugColor = strtoul(colorStr + strlen("color: "), nullptr, 0);
				} 	
			} else if (const char* linePtsStr = strMatch(str, "line: ")) {
                LinePts linePts;
                size_t recordSize = playbackBytes(&linePtsStr, (uint8_t*) &linePts, sizeof(linePts));
                if (!recordSize)
                    return noMatch("line");
                if (recordSize != sizeof(linePts))
                    return noMatch("line record size: expected: " + STR(sizeof(linePts)) + 
                            "; got: " + STR(recordSize));
                OpDebugRay line(linePts);
                lines.push_back(line);
            } else if (const char* curveStr = strMatch(str, "curveType: ")) {
                PathOpsV0Lib::ColorCurve curve;
                size_t typeSz = playbackBytes(&curveStr, (uint8_t*) &curve.curve.type, 
                        sizeof(curve.curve.type));
                if (sizeof(curve.curve.type) != typeSz)
                    return noMatch("curve type: " + STR(typeSz));
                curveStr = strMatch(curveStr, "curveData: ");
                if (!curveStr)
                    return noMatch("curveData");
                curve.curve.size = playbackBytes(&curveStr, nullptr, 0);
                if (!curve.curve.size)
                    return noMatch("curve");
                curve.curve.data = (PathOpsV0Lib::CurveData*) debugGlobalContext
                        ->allocateCallerData(curve.curve.size);
                playbackBytes(&curveStr, (uint8_t*) curve.curve.data, 0);
                curveStr = strMatch(curveStr, "color: ");
                if (!curveStr)
                    return noMatch("color");
                curve.color = strtoul(curveStr, nullptr, 0);
                curves.push_back(curve);
            } else if (const char* ptTStr = strMatch(str, "ptT: ")) {
                OpPtT ptT;
                size_t recordSize = playbackBytes(&ptTStr, (uint8_t*) &ptT, sizeof(ptT));
                if (!recordSize)
                    return noMatch("ptT");
                if (recordSize != sizeof(ptT))
                    return noMatch("ptT record size: expected: " + STR(sizeof(ptT)) + 
                            "; got: " + STR(recordSize));
                ptTs.push_back(ptT);
            } else if (0 == strcmp("brief\n", str)) {
				break;
			} else
				return noMatch(str);
		}
		redraw();
	}
}
#endif

#undef READ_FEATURE

void OpDebugImage::clearScreen() {
	SkCanvas offscreen(bitmap);
	offscreen.clear(SK_ColorWHITE);
}

void OpDebugImage::drawPath(const SkPath& path, uint32_t color) {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	paint.setColor(color);
	offscreen.drawPath(path, paint);
}

#if 0
void DebugColorEdges() {
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) {
		OpEdge* edge = *edgeIter;
		if (edge->disabled)
			edge->debugColor = red;
		else if (edge->inOutput)
			edge->debugColor = orange;
		else if (Unsortable::none != edge->isUnsortable)
			edge->debugColor = purple;
		else if (edgeIter.isCurveCurve) {
			if (edge->ccOverlaps) {
				PathOpsV0Lib::DebugOperand dbgOp = edge->segment->contour->debugCallbacks.
                    debugOperandFuncPtr;
				edge->debugColor = dbgOp && (*dbgOp)(edge->segment->contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData], 1)
						? orange : darkGreen;
			} else
				edge->debugColor = purple;
		} else
			edge->debugColor = black;
	}
}
#endif

void OpDebugImage::drawDoubleFocus() {
#if 0
	OP_DEBUG_CODE(OpDebugDefeatDelete defeater);
	std::vector<int> ids;
	clearScreen();
	{
		SkMatrix matrix;
		float scale = (float)DebugOpGetZoomScale();
		matrix.setScale(scale, scale);
		matrix.preTranslate((float) -DebugOpGetCenterX(), (float) -DebugOpGetCenterY());
		matrix.postTranslate((float) DebugOpGetOffsetX(), (float) DebugOpGetOffsetY());
		bool first = true;
		int alpha = drawFillOn ? 10 : 20;
		for (auto contour : contourIterator) {
			PathOpsV0Lib::DebugGetDraw debugGetDraw = contour->debugCallbacks.debugGetDrawFuncPtr;
			if (debugGetDraw && (*debugGetDraw)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData])) {
				PathOpsV0Lib::DebugNativePath debugNativePath = contour->debugCallbacks.
                    debugNativePathFuncPtr;
				if (!debugNativePath)
					continue;
				SkPath* skPath = (SkPath*) (*debugNativePath)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData]);
				drawDoubleFill(skPath->makeTransform(matrix), 
						first ? OpDebugAlphaColor(alpha, red) : OpDebugAlphaColor(alpha, blue));
			}
			first = false;
		}
#if 0
        if (drawResultOn && debugGlobalContext->callerOutput)
			drawDoubleFill(((SkPath*) debugGlobalContext->callerOutput)
					->makeTransform(matrix), OpDebugAlphaColor(alpha, green));
#endif
	}
	if (drawLinesOn)
		DebugOpDraw(lines);
	if (drawCurvesOn)
		DebugOpDraw(curves);
//	DebugColorEdges(); // set up default colors for all edges
	if (drawBoundsOn) {
		std::vector<OpRect> bounds;
		if (drawContoursOn) {
			for (auto contour : contourIterator)
				bounds.push_back(contour->bounds);
		}
		if (drawSegmentsOn) {
			for (auto segment : segmentIterator)
				bounds.push_back(segment->ptBounds);
		}
		if (drawEdgesOn) {
			for (auto edge : edgeIterator)
				bounds.push_back(edge->bounds);
		}
		DebugOpDraw(bounds);
	}
	if (drawRaysOn) {
		std::vector<OpDebugRay> rays;
		for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			const SectRay& sectRay = edge->ray;
			if (!sectRay.distances.size())
				continue;
			rays.emplace_back(sectRay.axis, sectRay.normal);
		}
		DebugOpDraw(rays);
	}
	if (drawSegmentsOn) {
		DebugOpClearSegments();
		for (auto segment : segmentIterator)
			DebugOpAdd(segment);
		DebugOpDrawSegments();
	}
	DebugOpClearEdges();
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) {
		const OpEdge* edge = *edgeIter;
		if (!edge->debugDraw)
			continue;
		DebugOpAdd(edge);
	}
	DebugOpDrawEdges();
	if (drawPointsOn)
		OpDebugImage::drawPoints();
	if (drawSegmentsOn && drawIDsOn) {
		for (auto segment : segmentIterator)
			DebugOpDrawSegmentID(segment, ids);
	}
	if (drawContoursOn && drawIDsOn) {
		for (auto contour : contourIterator) {
			DebugOpDrawContourID(contour, ids);
		}
	}
	if (drawIDsOn || drawNormalsOn || drawTangentsOn
			|| drawWindingsOn || drawEndToEndOn || drawControlLinesOn) {
		for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
				continue;
			ids.push_back(edge->id);
			uint32_t color = debugBlack; // edge->debugColor;
			if (drawIDsOn) {
				DebugOpDrawEdgeID(edge, color, drawLimbsOn);
			}
			if (drawNormalsOn)
				DebugOpDrawEdgeNormal(edge, color);
			if (drawTangentsOn)
				DebugOpDrawEdgeTangent(edge, color);
			if (drawWindingsOn)
				DebugOpDrawEdgeWinding(edge, color);
			if (drawEndToEndOn)
				DebugOpDrawEdgeEndToEnd(edge, OpDebugAlphaColor(40, color));
			if (drawControlLinesOn)
				DebugOpDrawEdgeControlLines(edge, OpDebugAlphaColor(40, color));
		}
	}
	if (drawIntersectionsOn && drawIDsOn) {
		for (auto sect : intersectionIterator)
			DebugOpDrawIntersectionID(sect, ids);
	}
	if (drawSegmentsOn && drawCoincidencesOn) {
		for (auto segment : segmentIterator)
			DebugOpDrawPointID(segment, ids);
	}
	if (drawSegmentsOn && (drawNormalsOn || drawTangentsOn || drawEndToEndOn || drawControlLinesOn)) {
		for (auto segment : segmentIterator) {
			if (drawNormalsOn)
				DebugOpDrawSegmentNormal(segment, segment->debugColor);
			if (drawTangentsOn)
				DebugOpDrawSegmentTangent(segment, segment->debugColor);
			if (drawEndToEndOn)
				DebugOpDrawSegmentEndToEnd(segment, OpDebugAlphaColor(40, segment->debugColor));
			if (drawControlLinesOn)
				DebugOpDrawSegmentControlLines(segment, OpDebugAlphaColor(40, segment->debugColor));
		}
	}
	if (drawCurvesOn && (drawNormalsOn || drawTangentsOn || drawEndToEndOn || drawControlLinesOn)) {
		for (auto curve : curves) {
			if (drawNormalsOn)
				DebugOpDrawCurveNormal(curve, red);
			if (drawTangentsOn)
				DebugOpDrawCurveTangent(curve, red);
			if (drawEndToEndOn)
				DebugOpDrawCurveEndToEnd(curve, OpDebugAlphaColor(40, red));
			if (drawControlLinesOn)
				DebugOpDrawCurveControlLines(curve, OpDebugAlphaColor(40, red));
		}
	}
	if (drawGridOn)
		drawGrid();
	if (drawRasterOn)
		drawRaster();
#endif
}

#if 0
static void recordBytes(FILE* recordFile, std::string text, uint8_t* data, size_t size) {
    std::string label = text  + ": ";
    fprintf(recordFile, "%s", label.c_str());
    for (size_t index = 0; index < size; ++index) {
        std::string byte = OpDebugByteToHex(data[index]) + " ";
        fprintf(recordFile, "%s", byte.c_str());
    }
}

void OpDebugImage::record(FILE* recordFile) {
	if (!recordFile) {
		OpDebugOut("failed to open opDebugImageState.txt for writing\n");
		return;
	}
	DebugOpRecord(recordFile);
	fprintf(recordFile, "textSize: %g\n", labelFont.getSize());
	fprintf(recordFile, "gridIntervals: %d\n", gridIntervals);
	fprintf(recordFile, "debugPrecision: %d\n", debugPrecision);
	fprintf(recordFile, "debugSmall: %d\n", debugSmall);
	fprintf(recordFile, "debugEpsilon: %d\n", debugEpsilon);
#define OP_X(Thing) \
	if (draw##Thing##On) \
		fprintf(recordFile, "%s\n", #Thing);
	MASTER_LIST
	EDGE_BOOL_LIST
	ALIAS_LIST
	CALLOUT_LIST
#undef OP_X
	for (auto e : edgeIterator) {
        if (!e->debugOne)
            continue;
		fprintf(recordFile, "edge: %d draw: %d color: 0x%08x\n", e->id, (int) e->debugDraw,
                e->debugColor);
	}
    for (OpDebugRay& line : lines) {
        recordBytes(recordFile, "line", (uint8_t*) &line.pts.pts.front(), sizeof(line.pts.pts));
		fprintf(recordFile, "\n");
    }
    for (PathOpsV0Lib::ColorCurve& curve : curves) {
        recordBytes(recordFile, "curveType", (uint8_t*) &curve.curve.type, sizeof(curve.curve.type));
        recordBytes(recordFile, "curveData", (uint8_t*) curve.curve.data, curve.curve.size);
		fprintf(recordFile, "color: 0x%08x\n", curve.color);
    }
     for (OpPtT& ptT : ptTs) {
        recordBytes(recordFile, "ptT", (uint8_t*) &ptT, sizeof(ptT));
		fprintf(recordFile, "\n");
    }
//	fclose(recordFile);
}
#endif

#undef RECORD_FEATURE

void OpDebugImage::drawDoubleCenter(OpPoint pt, bool add) {
	add ? DebugOpAddBounds(pt.x, pt.y, pt.x, pt.y) :	// !!! should suppress zoom change
			DebugOpSetCenter(pt.x, pt.y);
	drawDoubleFocus();
}

void OpDebugImage::drawDoubleFocus(const OpRect& b, bool add) {
	add ? DebugOpAddBounds(b.left, b.top, b.right, b.bottom) :
			DebugOpSetBounds(b.left, b.top, b.right, b.bottom);
	drawDoubleFocus();
}

void OpDebugImage::drawDoubleFill(const SkPath& path, uint32_t color, bool strokeAndFill) {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(color);
	if (strokeAndFill) {
		offscreen.drawPath(path, paint);
		paint.setColor(SK_ColorBLACK);
	}
	paint.setStyle(SkPaint::kFill_Style);
	offscreen.drawPath(path, paint);
}

void OpDebugImage::drawDoublePath(const SkPath& path, uint32_t color, float strokeWidth) {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(color);
	if (strokeWidth < 0) {
		offscreen.drawPath(path, paint);
		paint.setColor(SK_ColorBLACK);
	}
	paint.setStyle(SkPaint::kStroke_Style);
	if (strokeWidth > 0)
		paint.setStrokeWidth(strokeWidth);
	offscreen.drawPath(path, paint);
}

void OpDebugImage::drawGrid() {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(0x3f000000);
	SkPaint textPaint = paint;
	paint.setStyle(SkPaint::kStroke_Style);
	int xOffset = 2;
	double left, top, right, bottom;
	DebugOpBounds(left, top, right, bottom);
	auto fixSign = [](int32_t i) {
		return i < 0 ? -(i & 0x7fffffff) : i;
	};
	auto unfixSign = [](int32_t i) {
		return i < 0 ? -i | 0x80000000 : i;
	};
	int32_t leftH = fixSign(OpDebugFloatToBits((float) left));
	int32_t topH = fixSign(OpDebugFloatToBits((float) top));
	int32_t rightH = fixSign(OpDebugFloatToBits((float) right));
	int32_t bottomH = fixSign(OpDebugFloatToBits((float) bottom));
	int xInterval = std::max(1, (rightH - leftH) / gridIntervals);
	int yInterval = std::max(1, (bottomH - topH) / gridIntervals);
	int leftS, topS, rightS, bottomS;
	DebugOpScreenBounds(leftS, topS, rightS, bottomS);
	auto screenX = [leftS, left, rightS, right](float fx) {
		return leftS + (fx - left) / (right - left) * (rightS - leftS);
	};
	auto drawXLine = [screenX, &offscreen, &paint, &textPaint, topS, bottomS, xOffset](float fx) {
		float sx = (float) screenX(fx);
		offscreen.drawLine(sx, (float) topS, sx, (float) bottomS, paint);
		if (!drawValuesOn)
			return;
		std::string xValStr = drawHexOn ? OpDebugDumpHex(fx) : STR(fx);
		offscreen.drawString(SkString(xValStr), (float) (sx + xOffset), (float) (bitmapWH - xOffset - 3), 
				labelFont, textPaint);

	};
	auto walkX = [drawXLine, unfixSign, leftH, rightH, xInterval](bool preflight) {
		for (int x = leftH; x <= rightH; x += xInterval) {
			float fx = OpDebugBitsToFloat(unfixSign(x));
			if (preflight) {
				if (fabsf(fx) < 1)
					return false;
			} else
				drawXLine(fx);
		}
		return true;
	};
	bool xHexWorks = walkX(true);
	if (xHexWorks)
		walkX(false);
	else {	// if fabsf(fx) is less than 1, step by float range / gridIntervals
		for (float fx = (float) left; fx <= (float) right; 
				fx += (float) ((right - left) / (gridIntervals - 1))) {
			drawXLine(fx);
		}
	}
	auto screenY = [topS, top, bottomS, bottom](float fy) {
		return topS + (fy - top) / (bottom - top) * (bottomS - topS);
	};
	auto drawYLine = [screenY, &offscreen, &paint, &textPaint, leftS, rightS, xOffset]
			(float fy, bool last) {
		float sy = (float) screenY(fy);
		offscreen.drawLine((float) leftS, sy, (float) rightS, sy, paint);
		if (!drawValuesOn)
			return;
		std::string yValStr = drawHexOn ? OpDebugDumpHex(fy) : STR(fy);
		offscreen.save();
		if (last)
			sy -= 14;
		offscreen.rotate(-90, 15, sy - xOffset);
		offscreen.drawString(SkString(yValStr), 15, sy - xOffset, labelFont, textPaint);
		offscreen.restore();
	};
	auto walkY = [drawYLine, unfixSign, topH, bottomH, yInterval](bool preflight) {
		for (int y = topH; y <= bottomH; y += yInterval) {
			float fy = OpDebugBitsToFloat(unfixSign(y));
			if (preflight) {
				if (fabsf(fy) < 1)
					return false;
			} else
				drawYLine(fy, y > bottomH - yInterval);
		}
		return true;
	};
	bool yHexWorks = walkY(true);
	if (yHexWorks)
		walkY(false);
	else {	// if fabsf(fy) is less than 1, step by float range / gridIntervals
		float fInterval = (float) ((bottom - top) / (gridIntervals - 1));
		for (float fy = (float) top; fy <= (float) bottom; fy += fInterval) {
			drawYLine(fy, fy > bottom - fInterval);
		}
	}
}

// !!! should be named 'move grid center to' ? or should it keep center while zooming?
void gridCenter(int x, int y) {
	if (x < 0 || x >= gridIntervals || y < 0 || y >= gridIntervals)
		return OpDebugOut("parameters must be in grid interval range: 0 to " + STR(gridIntervals) + "\n");
	double left, top, right, bottom;
	DebugOpBounds(left, top, right, bottom);
	OpPoint center((float) (left + (right - left) * (gridIntervals - x) / gridIntervals),
			(float) (top + (bottom - top) * (gridIntervals - y) / gridIntervals));
	OpDebugImage::drawDoubleCenter(center, false);
}

void gridLines(int intervals) {
	gridIntervals = intervals;
	OpDebugImage::drawDoubleFocus();
}

void gridStep(float dxy) {
	double left, top, right, bottom;
	DebugOpBounds(left, top, right, bottom);
	double currentStep = (right - left) / gridIntervals;
	double newZoom = DebugOpGetZoomScale() * currentStep / dxy;
	DebugOpSetZoomScale(newZoom);
	// !!! OP_ASSERT bounds is square?
	OpDebugImage::drawDoubleFocus();
}

void precision(int p) {
	debugPrecision = p;
	OpDebugImage::drawDoubleFocus();
}

void smallFloats(bool sm) {
	debugSmall = sm;
	OpDebugImage::drawDoubleFocus();
}

void showEpsilon(bool sh) {
	debugEpsilon = sh;
	OpDebugImage::drawDoubleFocus();
}

void redraw() {
	OpDebugImage::drawDoubleFocus();
}

void textSize(float s) {
	labelFont.setSize(s);
	OpDebugImage::drawDoubleFocus();
}

void OpDebugImage::center(int id, bool add) {
	OpPointBounds pointBounds;
	OpPoint point;
	if (!find(id, &pointBounds, &point))
		return;
	if (pointBounds.isFinite())
		return OpDebugImage::drawDoubleCenter(pointBounds.center(), add);
	if (point.isFinite())
		return OpDebugImage::drawDoubleCenter(point, add);
}

void OpDebugImage::centerT(int id, float t) {
	if (OpEdge* edge = findEdge(id))
		OpDebugImage::drawDoubleCenter(edge->curve.ptAtT(t), false);
}

bool OpDebugImage::find(int id, OpPointBounds* boundsPtr, OpPoint* pointPtr) {
	if (std::vector<const OpIntersection*> coins = findCoincidence(id); coins.size()) {
		DRAW_IDS_ON(Coincidences);
		// !!! wrong: add rect formed by both intersections with this id
		for (auto coin : coins)
			boundsPtr->add(coin->ptT.pt);
		return true;
	}
	if (OpEdge* edge = findEdge(id)) {
		edge->debugDraw = true;
//		edge->debugOne = true;
		drawIDsOn = true;
		*boundsPtr = edge->bounds;
		return true;
	}
	if (std::vector<const OpEdge*> matches = findEdgeRayMatch(id); matches.size()) {
		drawIDsOn = true;
		for (auto match : matches)
			boundsPtr->add(match->bounds);
		return true;
	}
	if (const OpIntersection* intersection = findIntersection(id)) {
		DRAW_IDS_ON(Intersections);
		// don't change zoom
		*pointPtr = intersection->ptT.pt;
		return true;
	}
	if (const OpLimb* limb = findLimb(id)) {
		limb->edge->debugDraw = true;
//		limb->edge->debugOne = true;
		drawIDsOn = true;
		*boundsPtr = limb->edge->bounds;
		return true;
	}
	if (std::vector<const OpIntersection*> uSects = findSectUnsectable(id); uSects.size()) {
		// !!! wrong: add rect formed by both intersections with this id
		for (auto uSect : uSects)
			boundsPtr->add(uSect->ptT.pt);
		return true;
	}
	if (const OpSegment* segment = findSegment(id)) {
		DRAW_IDS_ON(Segments);
		*boundsPtr = segment->ptBounds;
		return true;
	}
	OpDebugOut("id " + STR(id) + " not found\n");
	return false;
}

void OpDebugImage::focus(int id, bool add) {
	OpPointBounds pointBounds;
	OpPoint point;
	if (!find(id, &pointBounds, &point))
		return;
	if (pointBounds.isFinite())
		return OpDebugImage::drawDoubleFocus(pointBounds, add);
	if (point.isFinite())
		return OpDebugImage::drawDoubleCenter(point, add);
}

void addFocus(int id) {
	OpDebugImage::focus(id, true);
}

void addFocus(const LinePts& linePts) {
	OpPointBounds bounds;
	bounds.add(linePts.pts[0]);
	bounds.add(linePts.pts[1]);
	addFocus(bounds);
}

void addFocus(const OpContour& contour) {
	for (const OpSegment& s : contour.segments)
		addFocus(s.ptBounds);
}

void addFocus(OpContext& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		addFocus(contour);
	addFocus(bounds);
}

// !!! remove all const everywhere
void addFocus(const OpContext& contours) {
	addFocus(*const_cast<OpContext*>(&contours));
}

void addFocus(const OpCurve& curve) {
	addFocus(curve.ptBounds());
}

void addFocus(const OpEdge& edge) {
	addFocus(edge.bounds);
}

void addFocus(const OpIntersection& sect) {
	addFocus(sect.ptT);
}

void addFocus(const OpPoint& pt) {
	OpDebugImage::drawDoubleCenter(pt, true);
}

void addFocus(const OpPtT& ptT) {
	addFocus(ptT.pt);
}

void addFocus(const OpRect& rect) {
	OpDebugImage::drawDoubleFocus(rect, true);
}

void addFocus(const OpSegment& segment) {
	addFocus(segment.ptBounds);
}

void addFocus(const PathOpsV0Lib::AddCurve& c) {
	OpCurve curve(c, Rotated::debug);
	addFocus(curve);
}

void addFocus(const PathOpsV0Lib::Curve& c) {
	OpCurve curve(c, Rotated::debug);
	addFocus(curve);
}

void addFocus(const OpContour* contour) {
	addFocus(*contour);
}

void addFocus(const OpContext* context) {
	addFocus(*context);
}

void addFocus(const OpCurve* curve) {
	addFocus(*curve);
}

void addFocus(const OpEdge* edge) {
	addFocus(*edge);
}

void addFocus(const OpIntersection* sect) {
	addFocus(*sect);
}

void addFocus(const OpPoint* pt) {
	addFocus(*pt);
}

void addFocus(const OpPtT* ptT) {
	addFocus(*ptT);
}

void addFocus(const OpRect* rect) {
	addFocus(*rect);
}

void addFocus(const OpSegment* segment) {
	addFocus(*segment);
}

void addFocus(const PathOpsV0Lib::AddCurve* c) {
	addFocus(*c);
}

void addFocus(const PathOpsV0Lib::Curve* c) {
	addFocus(*c);
}

void ctr() {
	ctr(*debugGlobalContext);
}

void ctr(int id) {
	OpDebugImage::center(id, false);
}

void ctr(float x, float y) {
	ctr(OpPoint(x, y));
}

void ctr(const LinePts& linePts) {
	OpPointBounds bounds;
	bounds.add(linePts.pts[0]);
	bounds.add(linePts.pts[1]);
	ctr(bounds);
}

void ctr(const OpContour& contour) {
	OpPointBounds bounds;
	for (auto& segment : contour.segments)
		bounds.add(segment.ptBounds);
	ctr(bounds);
}

void ctr(OpContext& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		for (auto& segment : contour->segments)
			bounds.add(segment.ptBounds);
	ctr(bounds);
}

void ctr(const OpContext& contours) {
	ctr(*const_cast<OpContext*>(&contours));
}

void ctr(const OpCurve& curve) {
	ctr(curve.ptBounds());
}

void ctr(const OpEdge& edge) {
	ctr(edge.bounds);
}

void ctr(const OpIntersection& sect) {
	ctr(sect.ptT);
}

void ctr(const OpPoint& pt) {
	OpDebugImage::drawDoubleCenter(pt, false);
}

void ctr(const OpPtT& ptT) {
	ctr(ptT.pt);
}

void ctr(const OpRect& rect) {
	ctr(rect.center());
}

void ctr(const OpSegment& segment) {
	ctr(segment.ptBounds);
}

void ctr(const PathOpsV0Lib::AddCurve& c) {
	OpCurve curve(c, Rotated::debug);
	ctr(curve);
}

void ctr(const PathOpsV0Lib::Curve& c) {
	OpCurve curve(c, Rotated::debug);
	ctr(curve);
}

void ctr(const OpContour* contour) {
	ctr(*contour);
}

void ctr(const OpContext* context) {
	ctr(*context);
}

void ctr(const OpCurve* curve) {
	ctr(*curve);
}

void ctr(const OpEdge* edge) {
	ctr(*edge);
}

void ctr(const OpIntersection* sect) {
	ctr(*sect);
}

void ctr(const OpPoint* pt) {
	ctr(*pt);
}

void ctr(const OpPtT* ptT) {
	ctr(*ptT);
}

void ctr(const OpRect* rect) {
	ctr(*rect);
}

void ctr(const OpSegment* segment) {
	ctr(*segment);
}

void ctr(const PathOpsV0Lib::AddCurve* c) {
	ctr(*c);
}

void ctr(const PathOpsV0Lib::Curve* c) {
	ctr(*c);
}

void ctrT(int id, float t) {
	OpDebugImage::centerT(id, t);
}

void ctrT(const OpEdge& edge, float t) {
	ctr(edge.id, t);
}

void ctrT(const OpEdge* edge, float t) {
	ctrT(*edge, t);
}


void focus(int id) {
	OpDebugImage::focus(id, false);
}

void focus(const LinePts& linePts) {
	OpPointBounds bounds;
	bounds.add(linePts.pts[0]);
	bounds.add(linePts.pts[1]);
	focus(bounds);
}

void focus(const OpContour& contour) {
	drawSegmentsOn = true;
	OpPointBounds bounds;
	for (auto& segment : contour.segments)
		bounds.add(segment.ptBounds);
	focus(bounds);
}

void focus(OpContext& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		for (auto& segment : contour->segments)
			bounds.add(segment.ptBounds);
	focus(bounds);
}

// !!! remove all const everywhere
void focus(const OpContext& contours) {
	focus(*const_cast<OpContext*>(&contours));
}

void focus(const OpCurve& curve) {
	focus(curve.ptBounds());
}

void focus(const OpEdge& edge) {
	focus(edge.bounds);
}

void focus(const OpRect& rect) {
	OpDebugImage::drawDoubleFocus(rect, false);
}

void focus(const OpSegment& segment) {
	drawSegmentsOn = true;
	focus(segment.ptBounds);
}

void focus(const PathOpsV0Lib::AddCurve& c) {
	OpCurve curve(c, Rotated::debug);
	focus(curve);
}

void focus(const PathOpsV0Lib::Curve& c) {
	OpCurve curve(c, Rotated::debug);
	focus(curve);
}

void focus(const OpContour* contour) {
	focus(*contour);
}

void focus(const OpContext* context) {
	focus(*context);
}

void focus(const OpCurve* curve) {
	focus(*curve);
}

void focus(const OpEdge* edge) {
	focus(*edge);
}

void focus(const OpRect* rect) {
	focus(*rect);
}

void focus(const OpSegment* segment) {
	focus(*segment);
}

void focus(const PathOpsV0Lib::AddCurve* c) {
	focus(*c);
}

void focus(const PathOpsV0Lib::Curve* c) {
	focus(*c);
}

void OpDebugImage::focusEdges() {
	if (edgeIterator.empty())
		return;
	OpPointBounds focusRect;
	for (auto edge : edgeIterator) {
		if (!edge->debugDraw)
			continue;
		focusRect.add(edge->bounds);
	}
	drawIDsOn = true;
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

void focusLinkInner(const OpEdge& edge, bool add) {
	add ? addFocus(edge) : focus(edge);
	const OpEdge* looped = edge.debugIsLoop(EdgeMatch::start, LeadingLoop::in);
	bool firstLoop = false;
	int safetyCount = 0;
	for (EdgeMatch which : { EdgeMatch::start, EdgeMatch::end } ) {
		const OpEdge* link = &edge;
		while ((link = EdgeMatch::start == which ? link->priorEdge : link->nextEdge)) {
			addFocus(link);
			if (link == looped) {
				if (firstLoop)
					return;
				firstLoop = true;
			}
			if (++safetyCount > 700) {
				OpDebugOut(std::string("!!! likely loops forever: ") + 
						(EdgeMatch::start == which ? "prior " : "next "));
				break;
			}
		}
	}
}

void focusLink(const OpEdge* edge) {
	focusLinkInner(*edge, false);
}

void focusLink(int id) {
	OpEdge* edge = findEdge(id);
	if (!edge) {
		OpDebugOut("not an edge id\n");
		return;
	}
	focusLink(edge);
}

void addFocusLink(const OpEdge& edge) {
	focusLinkInner(edge, true);
}

void addFocusLink(const OpEdge* edge) {
	addFocusLink(*edge);
}

void addFocusLink(int id) {
	OpEdge* edge = findEdge(id);
	if (!edge) {
		OpDebugOut("not an edge id\n");
		return;
	}
	addFocusLink(edge);
}

void focusCurves() {
	if (curves.empty())
		return;
	OpPointBounds focusRect = { curves.front().curve.data->start, curves.front().curve.data->end };
	for (auto& curve : curves) {
		focusRect.add(curve.curve.data->start);
		focusRect.add(curve.curve.data->end);
	}
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

void focusLines() {
	if (lines.empty())
		return;
	OpPointBounds focusRect = { lines.front().pts.pts[0], lines.front().pts.pts[1] };
	for (auto& line : lines) {
		focusRect.add(line.pts.pts[0]);
		focusRect.add(line.pts.pts[1]);
	}
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

void focusSegments() {
	if (segmentIterator.empty())
		return;
	OpPointBounds focusRect = (*segmentIterator.begin())->ptBounds;
	for (auto seg : segmentIterator)
		focusRect.add(seg->ptBounds);
	DRAW_IDS_ON(Segments);
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

// immediate window commands

// note that operands are not cleared
void clear() {
	OpDebugImage::clearScreen();
	OpDebugImage::clearCurves();
	OpDebugImage::clearLines();
	OpDebugImage::clearPoints();
	DebugOpResetFocus();
}

void clearCurves() {
	OpDebugImage::clearCurves();
	DebugOpResetFocus();
}

void clearLines() {
	OpDebugImage::clearLines();
	DebugOpResetFocus();
}

void clearPoints() {
	OpDebugImage::clearPoints();
	DebugOpResetFocus();
}

bool OpDebugImage::bitsAreBlank(bool allowIntersect, SkRect& test) {
	SkRect skBounds { 0, 0, bitmapWH, bitmapWH };
	if (allowIntersect ? !skBounds.intersect(test) : !skBounds.contains(test))
		return false;
	int left = std::max(0, (int) test.fLeft);
	int top = std::max(0, (int) test.fTop);
	int right = std::min(bitmapWH, (int) test.fRight);
	int bottom = std::min(bitmapWH, (int) test.fBottom);
	for (int y = top; y < bottom; ++y) {
		for (int x = left; x < right; ++x) {
			uint32_t pixel = bitmap.getColor(x, y);
			if (SkColorGetR(pixel) == SkColorGetG(pixel)
					&& SkColorGetR(pixel) == SkColorGetB(pixel)
					&& SkColorGetR(pixel) < 0xf0)
				return false;
			if (SkColorGetR(pixel) < 0x3F
					|| SkColorGetG(pixel) < 0x3F
					|| SkColorGetB(pixel) < 0x3F)
				return false;
		}
	}
	return true;
}

void OpDebugImage::drawGuide(const SkRect& test, OpPoint pt, uint32_t color) {
	// add line from edge of text box towards original point
	SkRect box = test;
	OpPoint closestSide;
	float closest = OpInfinity;
	for (int side = 0; side < 4; ++side) {
		OpPoint tSide = { 
				side & 1 ? box.centerX() : 0 == side ? box.fLeft : box.fRight,
				side & 1 ? 1 == side ? box.fTop : box.fBottom : box.centerY()
		};
		float distance = (tSide - pt).length();
		if (closest > distance) {
			closest = distance;
			closestSide = tSide;
		}
	}
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(color);
	paint.setAlpha(63);
	SkCanvas offscreen(bitmap);
	offscreen.drawLine(closestSide.x, closestSide.y, pt.x, pt.y, paint);
}

void OpDebugImage::drawRaster() {
	OpPointBounds bounds = debugGlobalContext->maxBounds;
	if (bounds.width() > bounds.height())
		bounds.bottom = bounds.top + bounds.width();
	else
		bounds.right = bounds.left + bounds.height();
	OpPoint mapLT = DebugOpPtToPt(OpPoint(bounds.left, bounds.top));
	OpPoint mapBR = DebugOpPtToPt(OpPoint(bounds.right, bounds.bottom));
	SkPath path;
	float x = mapLT.x;
	float y = mapLT.y;
	float xStep = (mapBR.x - mapLT.x) / rasterIntervals;
	float yStep = (mapBR.y - mapLT.y) / rasterIntervals;
	for (int index = 0; index <= rasterIntervals; ++index) {
		path.moveTo(x, mapLT.y);
		path.lineTo(x, mapBR.y);
		x += xStep;
		path.moveTo(mapLT.x, y);
		path.lineTo(mapBR.x, y);
		y += yStep;
	}
	OpDebugImage::drawPath(path, 0x3f000000);
#if 0 && TEST_RASTER
	float rW = xStep / 2;
	float rH = yStep / 2;
	auto draw = [rW, rH, mapLT, xStep, yStep](OpDebugRaster* raster) {
		if (!raster)
			return SkPath();
		if (RasterType::unset == raster->type)
			return SkPath();
		SkPath path;
		uint8_t* bitsPtr = raster->bits;
		float rT = mapLT.y;
		if (RasterType::combined == raster->type || RasterType::op == raster->type)
			rT += rH;
		for (int y = 0; y < raster->bitHeight; ++y) {
			float rL = mapLT.x;
			if (RasterType::right == raster->type || RasterType::op == raster->type)
				rL += rW;
			for (int x = 0; x < raster->bitWidth; ++x) {
				if (!*bitsPtr++)
					path.addRect(rL, rT, rL + rW, rT + rH);
				rL += xStep;
			}
			rT += yStep;
		}
		return path;
	};
	SkPath lPath = draw(debugGlobalContext->debugData.leftRaster);
	OpDebugImage::drawDoubleFill(lPath, 0x7fff0000);
	SkPath rPath = draw(debugGlobalContext->debugData.rightRaster);
	OpDebugImage::drawDoubleFill(rPath, 0x7f00ff00);
	SkPath cPath = draw(debugGlobalContext->debugData.combinedRaster);
	OpDebugImage::drawDoubleFill(cPath, 0x7f0000ff);
	SkPath oPath = draw(&debugGlobalContext->opRaster);
	OpDebugImage::drawDoubleFill(oPath, 0x7fff00ff);
#endif
}

bool OpDebugImage::drawValue(OpPoint pt, std::string ptStr, uint32_t color) {
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(color);
	SkCanvas offscreen(bitmap);
	SkRect textBounds;
	(void) labelFont.measureText(ptStr.c_str(), ptStr.length(), SkTextEncoding::kUTF8, &textBounds);
#if 0
   SkTypeface* typeface = labelFont.getTypefaceOrDefault();
    if (typeface) {
        SkString familyName;
        typeface->getFamilyName(&familyName);
        // Now 'familyName' contains the font's family name
        // You can print it or use it as needed
        printf("Font Family Name: %s\n", familyName.c_str());
    } else {
        printf("No typeface associated with this SkFont.\n");
    }
#endif
	const int xOffset = 2;
	const int yOffset = 1;
	textBounds.inset(-xOffset, -yOffset);
	SkRect trimmed = textBounds;
	trimmed.offset(pt.x, pt.y);
	for (bool allowIntersect : { false, true } ) {
		for (float scale : { 4.f, 16.f, 32.f, 48.f, 64.f } ) {
			for (int toTheLeft : { -1, 0, 1 } ) {
				for (int toTheTop : { -1, 0, 1 } ) {
					OpVector offset { 0, 0 };
					SkRect test = trimmed;
					if (toTheLeft)
						offset.dx = toTheLeft < 0 ? -textBounds.width() - scale : scale;
					if (toTheTop)
						offset.dy = toTheTop < 0 ? -scale : textBounds.height() + scale;
					test.offset(offset.dx, offset.dy);
					if (!bitsAreBlank(allowIntersect, test))
						continue;
					offscreen.drawString(SkString(ptStr), pt.x + offset.dx,
							pt.y + offset.dy, labelFont, paint);
					if (16 <= scale && drawGuidesOn)
						drawGuide(test, pt, color);
					return true;
				}
			}
		}
	}
	offscreen.drawString(SkString("."), pt.x, pt.y, labelFont, paint);
	return false;
}

#if 0
void OpDebugImage::drawPoints() {
	DebugOpClearPoints();
	auto drawPathPt = [](const SkPath* path) { // lambda
		SkPath::RawIter iter(*path);
		SkPath::Verb verb;
		do {
			SkPoint pts[4];
			verb = iter.next(pts);
			switch (verb) {
			case SkPath::kMove_Verb:
				break;
			case SkPath::kLine_Verb:
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY), black);
				DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY), black);
				break;
			case SkPath::kQuad_Verb:
			case SkPath::kConic_Verb:
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY), black);
				if (drawControlsOn)
					DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY), black);
				DebugOpBuild(OpPoint(pts[2].fX, pts[2].fY), black);
				break;
			case SkPath::kCubic_Verb:
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY), black);
				if (drawControlsOn) {
					DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY), black);
					DebugOpBuild(OpPoint(pts[2].fX, pts[2].fY), black);
				}
				DebugOpBuild(OpPoint(pts[3].fX, pts[3].fY), black);
				break;
			case SkPath::kClose_Verb:
			case SkPath::kDone_Verb:
				break;
			}
		} while (verb != SkPath::kDone_Verb);
	};
	for (auto contour : contourIterator) {
		PathOpsV0Lib::DebugGetDraw debugGetDraw = contour->debugCallbacks.debugGetDrawFuncPtr;
		if (debugGetDraw && (*debugGetDraw)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData])) {
			PathOpsV0Lib::DebugNativePath debugNativePath = contour->debugCallbacks.debugNativePathFuncPtr;
			if (!debugNativePath)
				continue;
			SkPath* skPath = (SkPath*) (*debugNativePath)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData]);
			drawPathPt(skPath);
		}
	}
	if (drawSegmentsOn) {
		for (auto seg : segmentIterator) {
			DebugOpBuild(seg->c.firstPt(), black);
			DebugOpBuild(seg->c.lastPt(), black);
			if (drawControlsOn) {
				for (int index = 1; index < seg->c.pointCount() - 1; ++index)
					DebugOpBuild(seg->c.hullPt(index), black);
			}
		}
	}
	for (auto edge : edgeIterator) {
		if (!edge->debugDraw)
			continue;
		OpContour* contour = edge->segment->contour;
		PathOpsV0Lib::DebugOperand debugIsOpp = contour->debugCallbacks.debugOperandFuncPtr;
		bool isOpp = debugIsOpp && (*debugIsOpp)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData], 1);
		DebugOpBuild(edge->curve.firstPt(), debugBlack, edge->startT, isOpp);
		DebugOpBuild(edge->curve.lastPt(), debugBlack, edge->endT, isOpp);
		if (drawControlsOn) {
			for (int index = 1; index < edge->curve.pointCount() - 1; ++index)
				DebugOpBuild(edge->curve.hullPt(index), debugBlack);
		}
		if (drawCentersOn)
			DebugOpBuild(edge->center.pt, debugBlack, edge->center.t, DebugSprite::square);
		if (drawHullsOn) {
			for (const HullSect& hull : edge->hulls.h)
				DebugOpBuild(hull.sect.pt, debugBlack, hull.sect.t, DebugSprite::circle);
		}
        if (drawIPointsOn) {
            if (edge->iStart != edge->curve.firstPt())
		        DebugOpBuild(edge->iStart, debugBlack, edge->startT, isOpp);
            if (edge->iEnd != edge->curve.lastPt())
		        DebugOpBuild(edge->iEnd, debugBlack, edge->endT, isOpp);
        }
	}
	if (drawIntersectionsOn) {
		for (const auto& sect : intersectionIterator) {
			DebugOpBuild(sect->ptT.pt, black);
		}
	}
	if (drawLinesOn) {
		for (const auto& line : lines) {
			DebugOpBuild(line.pts.pts[0], black);
			DebugOpBuild(line.pts.pts[1], black);
			for (auto contour : contourIterator) {
				PathOpsV0Lib::DebugGetDraw debugGetDraw = contour->debugCallbacks.debugGetDrawFuncPtr;
				if (debugGetDraw && (*debugGetDraw)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData])) {
					PathOpsV0Lib::DebugNativePath debugNativePath = contour->debugCallbacks.debugNativePathFuncPtr;
					if (!debugNativePath)
						continue;
					DebugOpBuild(*(SkPath*)debugNativePath(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData]), line);
				}
			}
			if (drawSegmentsOn) {
				for (auto seg : segmentIterator) {
					DebugOpBuild(*seg, line);
				}
			}
			for (auto edge : edgeIterator) {
				if (!edge->debugDraw)
					continue;
				DebugOpBuild(*edge, line);
			}
		}
	}
	if (drawCurvesOn) {
		for (const auto& curve : curves) {
			DebugOpBuild(curve);
			if (drawControlsOn) {
				OpCurve opCurve(curve.curve, Rotated::debug);
				for (int index = 1; index < opCurve.pointCount() - 1; ++index)
					DebugOpBuild(opCurve.hullPt(index), curve.color);
			}
			for (const auto& line : lines) {
				DebugCurveBuild(curve, line);
			}
		}
	}
	if (drawRaysOn) {
			for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			const SectRay& ray = edge->ray;
			for (auto dist : ray.distances)
				DebugOpBuild(ray.axis, ray.normal, dist.cept);
		}
	}
	for (OpPtT ptT : ptTs)
		DebugOpBuild(ptT.pt, black, ptT.t, DebugSprite::triangle);
	if (drawValuesOn) {
		if (drawTsOn)
			DebugOpDrawT(drawHexOn);
		else
			DebugOpDrawValue(drawHexOn);
	}
	DebugOpDrawSprites();
}
#endif

void OpDebugImage::add(Axis axis, float value) {
	lines.emplace_back(axis, value);
}

void OpDebugImage::add(const OpPtT& ptT) {
	ptTs.push_back(ptT);
}

void OpDebugImage::addArrowHeadToPath(const LinePts& line, SkPath& path) {
	const SkPoint arrow[2] { { -10, -6 }, { 6, 6 } };
	float radians = atan2f(line.pts[1].y - line.pts[0].y, line.pts[1].x - line.pts[0].x);
	float degrees = (radians * 180) / 3.14159265f;
	if (degrees < 0)
		degrees += 360;
	SkMatrix matrix;
	matrix.setRotate(degrees);
	SkPoint arrowCopy[2] = { arrow[0], arrow[1] };
	matrix.mapPoints(arrowCopy, 2);
	path.rLineTo(arrowCopy[0].fX, arrowCopy[0].fY);
	path.rLineTo(arrowCopy[1].fX, arrowCopy[1].fY);
}

void OpDebugImage::addCircleToPath(OpPoint pt, SkPath& path) {
	path.addCircle(pt.x, pt.y, 4);
}

void OpDebugImage::addDiamondToPath(OpPoint pt, SkPath& path) {
	SkPath diamond;
	diamond.moveTo( 4,  0);
	diamond.lineTo( 0,  4);
	diamond.lineTo(-4,  0);
	diamond.lineTo( 0, -4);
	diamond.close();
	diamond.offset(pt.x, pt.y);
	path.addPath(diamond);
}

void OpDebugImage::addSquareToPath(OpPoint pt, SkPath& path) {
	SkPath square;
	square.moveTo(-4, -4);
	square.lineTo( 4, -4);
	square.lineTo( 4,  4);
	square.lineTo(-4,  4);
	square.close();
	square.offset(pt.x, pt.y);
	path.addPath(square);
}

void OpDebugImage::addTriangleToPath(OpPoint pt, SkPath& path) {
	SkPath triangle;
	triangle.moveTo( 0,  -4);
	triangle.lineTo( 4,  4);
	triangle.lineTo(-4,  4);
	triangle.close();
	triangle.offset(pt.x, pt.y);
	path.addPath(triangle);
}

void OpDebugRay::construct(const LinePts& pts_) {
	pts = pts_;
	if (pts.pts[0].x == pts.pts[1].x) {
		axis = Axis::vertical;
		value = pts.pts[0].x;
		useAxis = true;
		return;
	}
	if (pts.pts[0].y == pts.pts[1].y) {
		axis = Axis::horizontal;
		value = pts.pts[0].y;
		useAxis = true;
		return;
	}
	useAxis = false;
}

void OpDebugImage::add(const OpDebugRay& ray) {
	lines.emplace_back(ray);
}

void OpDebugImage::add(const PathOpsV0Lib::AddCurve& curve) {
	curves.push_back({{ curve.context, (PathOpsV0Lib::CurveData*) curve.points, curve.size, curve.type }, black});
}

void OpDebugImage::add(const PathOpsV0Lib::Curve& curve) {
	curves.push_back({ curve, black });
}

void OpDebugImage::add(const PathOpsV0Lib::AddCurve& curve, uint32_t color) {
	curves.push_back({{ curve.context, (PathOpsV0Lib::CurveData*) curve.points, curve.size, curve.type }, color});
}

void OpDebugImage::add(const PathOpsV0Lib::Curve& curve, uint32_t color) {
	curves.push_back({ curve, color });
}

void OpDebugImage::add(const OpRect& r) {
	LinePts leftEdge { OpPoint(r.left, r.top), OpPoint(r.left, r.bottom) };
	LinePts topEdge { OpPoint(r.left, r.top), OpPoint(r.right, r.top) };
	LinePts rightEdge { OpPoint(r.right, r.top), OpPoint(r.right, r.bottom) };
	LinePts botomEdge { OpPoint(r.left, r.bottom), OpPoint(r.right, r.bottom) };
	lines.emplace_back(leftEdge);
	lines.emplace_back(topEdge);
	lines.emplace_back(rightEdge);
	lines.emplace_back(botomEdge);
}

void OpDebugImage::clearIntersections() {
	drawIntersectionsOn = false;
    OpDebugImage::drawDoubleFocus();
}

void OpDebugImage::clearCurves() {
	curves.clear();
	drawCurvesOn = false;
    OpDebugImage::drawDoubleFocus();
}

void OpDebugImage::clearLines() {
	lines.clear();
	drawLinesOn = false;
    OpDebugImage::drawDoubleFocus();
}

void OpDebugImage::clearPoints() {
	ptTs.clear();
	drawPointsOn = false;
    OpDebugImage::drawDoubleFocus();
}

#define OP_X(Thing) \
void hide##Thing() { \
	draw##Thing##On = false; \
	OpDebugImage::drawDoubleFocus(); \
} \
 \
void show##Thing() { \
	draw##Thing##On = true; \
	OpDebugImage::drawDoubleFocus(); \
} \
 \
void toggle##Thing() { \
	draw##Thing##On ^= true; \
	OpDebugImage::drawDoubleFocus(); \
}
MASTER_LIST
#undef OP_X

#define OP_X(Thing, edgeCheck) \
static void operateOn##Thing(std::function<void (OpEdge*)> fun) { \
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) { \
		OpEdge* edge = *edgeIter; \
		if (edgeCheck) \
			continue; \
		fun(edge); \
	} \
	OpDebugImage::drawDoubleFocus(); \
} \
 \
void hide##Thing() { \
	operateOn##Thing([](OpEdge* edge) { \
		edge->debugDraw = false; \
	}); \
	draw##Thing##On = false; \
} \
 \
void show##Thing() { \
	operateOn##Thing([](OpEdge* edge) { \
		edge->debugDraw = true; \
	}); \
	draw##Thing##On = true; \
} \
 \
void toggle##Thing() { \
	operateOn##Thing([](OpEdge* edge) { \
		edge->debugDraw ^= true; \
	}); \
	draw##Thing##On ^= true; \
}
EDGE_BOOL_LIST2
#undef OP_X

#if 0
static void doOperand(int operand, bool leftState) {
	for (OpContour* contour : contourIterator) {
		PathOpsV0Lib::DebugOperand debugOperand = contour->debugCallbacks.debugOperandFuncPtr;
		PathOpsV0Lib::DebugSetDraw debugSetDraw = contour->debugCallbacks.debugSetDrawFuncPtr;
		if (!debugOperand || !(*debugOperand)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData], operand) 
                    || !debugSetDraw)
			continue;
		(*debugSetDraw)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData], leftState);
		OpDebugImage::drawDoubleFocus();
	}
}

void hideLeft() {
	doOperand(0, (drawLeftOn = false));
}

void showLeft() {
	doOperand(0, (drawLeftOn = true));
}

void toggleLeft() {
	doOperand(0, (drawLeftOn ^= true));
}

void hideRight() {
	doOperand(1, (drawRightOn = false));
}

void showRight() {
	doOperand(1, (drawRightOn = true));
}

void toggleRight() {
	doOperand(1, (drawRightOn ^= true));
}
#endif

static void operateOnLimbEdges(std::function<void (OpEdge*)> fun) {
	const OpTree* tree = debugGlobalContext->debugTree;
	if (!tree)
		return;
	for (int index = 0; index < tree->totalUsed; ++index) {
		const OpLimb& limb = debugGlobalContext->debugNthLimb(index);
		OpEdge* edge = limb.edge;
		bool callFun = 0 >= limbsShown || limbsShown > index;
		if (callFun)
			fun(edge);
		else
			edge->debugDraw = false;
 //       edge->debugOne = false;
		std::vector<OpEdge*> visited;
		if (edge->priorEdge && !edge->debugIsLoop())
			edge = const_cast<OpEdge*>(edge->debugAdvanceToEnd(EdgeMatch::start));
		if (edge->lastEdge && edge != edge->lastEdge) {
			OpEdge* next = edge;
			while ((next = next->nextEdge)) {
				if (visited.end() != std::find(visited.begin(), visited.end(), next))
					break;
				if (callFun)
					fun(next);
				else
					next->debugDraw = false;
//                next->debugOne = false;
				visited.push_back(next);
			}
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void hideLimbs() {
	operateOnLimbEdges([](OpEdge* edge) {
		edge->debugDraw = false;
	});
	drawLimbsOn = false;
	OpDebugImage::drawDoubleFocus();
}

void showLimbs() {
	operateOnLimbEdges([](OpEdge* edge) {
		edge->debugDraw = true;
	});
	drawLimbsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void toggleLimbs() {
	operateOnLimbEdges([](OpEdge* edge) {
		edge->debugDraw ^= true;
	});
	drawLimbsOn ^= true;
	OpDebugImage::drawDoubleFocus();
}

void limbs(int limit) {
	limbsShown = limit;
	showLimbs();
}

void hideTree() {
	hideLimbs();
}

void showTree() {
	showLimbs();
}

void toggleTree() {
	toggleLimbs();
}

// !!! could macro-tize this if common (note that disabled and linkups are nearly identical)
#if 0
void hideOperands() {
	for (auto contour : contourIterator) {
		PathOpsV0Lib::DebugSetDraw debugSetDraw = contour->debugCallbacks.debugSetDrawFuncPtr;
		if (!debugSetDraw)
			continue;
		(*debugSetDraw)(contour->debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], false);
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn = false;
}

void showOperands() {
	for (auto contour : contourIterator) {
		PathOpsV0Lib::DebugSetDraw debugSetDraw = contour->debugCallbacks.debugSetDrawFuncPtr;
		if (!debugSetDraw)
			continue;
		(*debugSetDraw)(contour->debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], true);
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn = true;
}

void toggleOperands() {
	for (auto contour : contourIterator) {
		PathOpsV0Lib::DebugSetDraw debugSetDraw = contour->debugCallbacks.debugSetDrawFuncPtr;
		PathOpsV0Lib::DebugGetDraw debugGetDraw = contour->debugCallbacks.debugGetDrawFuncPtr;
		if (!debugSetDraw || !debugGetDraw)
			continue;
		(*debugSetDraw)(contour->debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], 
                !(*debugGetDraw)(contour->debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData]));
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn ^= true;
}
#endif

static void operateOnID(std::function<void (OpEdge*)> fun, int id) {
	if (OpContour* contour = (OpContour*) findContour(id)) {
		for (OpSegment& segment : contour->segments) {
			for (OpEdge& edge : segment.edges) {
				fun(&edge);
			}
		}
	} else if (OpSegment* seg = (OpSegment*) findSegment(id)) {
		for (OpEdge& edge : seg->edges) {
			fun(&edge);
		}
    } else if (OpEdge* e = (OpEdge*) findEdge(id)) {
        fun(e);
//        e->debugOne = true;
    }
    OpDebugImage::drawDoubleFocus();
}

void show(int id) {
	operateOnID([](OpEdge* edge) {
		edge->debugDraw = true;
	}, id);
}

void hide(int id) {
	operateOnID([](OpEdge* edge) {
		edge->debugDraw = false;
	}, id);
}

void toggle(int id) {
	operateOnID([](OpEdge* edge) {
		edge->debugDraw ^= true;
	}, id);
}

#if 0
#define OP_X(Thing, edgeCheck) \
void color##Thing(uint32_t color) { \
	for (auto edge : edgeIterator) { \
		if (edgeCheck) { \
			edge->debugColor = color; \
			edge->debugDraw = true; \
			edge->debugOne = true; \
		} \
	} \
	OpDebugImage::drawDoubleFocus(); \
}
COLOR_LIST2
COLOR_DUP_LIST2
#undef OP_X

void colorContours(uint32_t color) {
	for (auto contour : contourIterator) {
		contour->debugColor = color;
	}
	OpDebugImage::drawDoubleFocus();
}

void colorSegments(uint32_t color) {
	for (auto seg : segmentIterator) {
		seg->debugColor = color;
	}
	OpDebugImage::drawDoubleFocus();
}

void colorLimbs(uint32_t color) {
	colorLimbRange(0, color);
}

void colorOpp(uint32_t color) {
	for (auto edge : edgeIterator) {
		OpContour* contour = edge->segment->contour;
		PathOpsV0Lib::DebugOperand debugOperand = contour->debugCallbacks.debugOperandFuncPtr;
		if (debugOperand && (*debugOperand)(contour->debugContourData[
                (size_t) PathOpsV0Lib::DebugContourType::windingUserData], 1)) {
			edge->debugColor = color;
			edge->debugDraw = true;
            edge->debugOne = true;
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void colorTree(uint32_t color) {
	colorLimbRange(0, color);
}

#define OP_X(Thing) \
void color##Thing() { \
	color##Thing(multiColor); \
} \
void color##Thing(uint8_t alpha, uint32_t color) { \
	color##Thing(OpDebugAlphaColor(alpha, color)); \
} \
void uncolor##Thing() { \
	color##Thing(0); \
}
COLOR_LIST
#undef OP_X

#define OP_X(Thing, edgeCheck) \
void show##Thing() { \
	for (auto edge : edgeIterator) { \
		if (edgeCheck) { \
			edge->debugDraw = true; \
            edge->debugOne = true; \
		} \
	} \
	OpDebugImage::drawDoubleFocus(); \
}
COLOR_LIST2
#undef OP_X

#define OP_X(Thing, edgeCheck) \
void hide##Thing() { \
	for (auto edge : edgeIterator) { \
		if (edgeCheck) { \
			edge->debugDraw = false; \
            edge->debugOne = true; \
		} \
	} \
	OpDebugImage::drawDoubleFocus(); \
}
COLOR_LIST2
#undef OP_X

#define OP_X(Thing, edgeCheck) \
void toggle##Thing() { \
	for (auto edge : edgeIterator) { \
		if (edgeCheck) { \
			edge->debugDraw ^= true; \
            edge->debugOne = true; \
		} \
	} \
	OpDebugImage::drawDoubleFocus(); \
}
COLOR_LIST2
#undef OP_X

void color(int id) {
	color(id, multiColor);
}

void color(int id, uint32_t c) {
	OpEdge* edge = findEdge(id);
	if (edge) {
		edge->debugColor = c;
		edge->debugDraw = true;
        edge->debugOne = true;
	} else if (OpContour* contour = (OpContour*) findContour(id)) {
		for (OpSegment& segment : contour->segments) {
			for (OpEdge& e : segment.edges) {
				e.debugColor = c;
				e.debugDraw = true;
                e.debugOne = true;
			}
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void color(int id, uint8_t a, uint32_t c) {
	color(id, OpDebugAlphaColor(a, c));
}

void color(PathOpsV0Lib::AddCurve& c, uint32_t clr) {
	OpDebugImage::add(c, clr);
	drawCurvesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void color(PathOpsV0Lib::AddCurve* c, uint32_t clr) {
    color(*c, clr);
}

void color(PathOpsV0Lib::Curve& c, uint32_t clr) {
	OpDebugImage::add(c, clr);
	drawCurvesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void color(PathOpsV0Lib::Curve* c, uint32_t clr) {
    color(*c, clr);
}

void uncolor(int id) {
	color(id, 0);
}

void colorLink(OpEdge* edge, uint32_t color) {
	auto colorChain = [edge, color](EdgeMatch which) {
		const OpEdge* looped = edge->debugIsLoop(which, LeadingLoop::in);
		bool firstLoop = false;
		int safetyCount = 0;
		OpEdge* chain = edge;
		for (;;) {
			chain->debugColor = color;
			chain->debugDraw = true;
            chain->debugOne = true;
			if (chain == looped) {
				if (firstLoop)
					return;
				firstLoop = true;
			}
			chain = EdgeMatch::start == which ? chain->priorEdge : chain->nextEdge;
			if (!chain)
				break;
			if (++safetyCount > 700) {
				OpDebugOut(std::string("!!! likely loops forever: ") + 
						(EdgeMatch::start == which ? "prior " : "next "));
				break;
			}
		}
	};
	colorChain(EdgeMatch::start);
	colorChain(EdgeMatch::end);
	OpDebugImage::drawDoubleFocus();
}

void colorLink(OpEdge& edge, uint32_t color) {
	colorLink(&edge, color);
}

void colorLink(int id, uint32_t color) {
	colorLink(findEdge(id), color);
}
#endif

void OpContext::debugLimbClear() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			for (OpEdge& edge : segment.edges) {
				edge.debugLimb = false;
			}
		}
	}
}

#if 0
void OpContext::debugLimbColor(int lastLimbID, uint32_t color) {
	if (!debugTree)
		return;
	for (int index = 0; index < debugTree->totalUsed; ++index) {
		const OpLimb& limb = debugNthLimb(index);
		if (!lastLimbID && limb.id > lastLimbID)
			continue;
		OpEdge* test = limb.edge;
		test->debugColor = color;
		test->debugDraw = true;
		test->debugOne = true;
	}
	OpDebugImage::drawDoubleFocus();
}

void colorLimbRange(int lastLimbID, uint32_t color) {
	debugGlobalContext->debugLimbColor(lastLimbID, color);
}
#endif

int OpContext::debugLimbIndex(const OpEdge* edge) const {
	if (!debugTree)
		return -1;
	for (int index = 0; index < debugTree->totalUsed; ++index) {
		const OpLimb& limb = debugNthLimb(index);
		const OpEdge* test = limb.edge;
		if (test == edge)
			return index;
		if (test->debugIsLoop())	// !!! conservative: may allow this later
			continue;
		if (!test->lastEdge)
			continue;
		do {
			if (test == edge)
				return index;
		} while ((test = test->nextEdge));
	}
	return -1;
}

#if 0
void OpEdge::color(uint32_t c) {
	debugColor = c;
	debugDraw = true;
    debugOne = true;
	OpDebugImage::drawDoubleFocus();
}
#endif

void OpEdge::addLink() {
	OpEdge* chain = this;
	std::vector<OpEdge*> seen;
	do {
		chain->debugDraw = true;
//        chain->debugOne = true;
		seen.push_back(chain);
	} while ((chain = chain->nextEdge) && seen.end() == std::find(seen.begin(), seen.end(), chain));
	chain = this;
	while ((chain = chain->priorEdge) && seen.end() == std::find(seen.begin(), seen.end(), chain)) {
		chain->debugDraw = true;
//        chain->debugOne = true;
		seen.push_back(chain);
	}
	drawIDsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void OpEdge::drawLink() {
	hideEdges();
	addLink();
}

void OpTree::debugLimbEdges(OpEdge* edge) {
	OP_ASSERT(!edge->debugIsLoop());
	OpEdge* first = edge;
	if (edge->priorEdge)
		first = const_cast<OpEdge*>(edge->debugAdvanceToEnd(EdgeMatch::start));
	OP_ASSERT(first->lastEdge || !first->nextEdge || edge->disabled);
	edge = first;
	do {
		edge->debugLimb = true;
	} while ((edge = edge->nextEdge));
}

#if 0
void OpCurveCurve::draw() const {
	if (!edgeCurves.c.size())
		return OpDebugOut("OpCurveCurve missing edgeCurves\n");
	OpPointBounds focusRect = edgeCurves.c.front()->ptBounds;
	for (auto edgesPtrs : { &edgeCurves.c, &oppCurves.c }) {
		for (auto& edge : *edgesPtrs)
			focusRect.add(edge->ptBounds);
	}
	DRAW_IDS_ON(Edges);
	OpDebugImage::drawDoubleFocus(focusRect, false);
}
#endif

bool OpDebugImage::drawEdgeNormal(OpVector norm, OpPoint midTPt, int edgeID, uint32_t color) {
	LinePts normal { midTPt, midTPt + norm };
	SkPath normalPath;
	normalPath.moveTo(normal.pts[0].x, normal.pts[0].y);
	if (!normal.pts[1].isFinite()) {
		OpDebugOut("normal not finite on edge " + STR(edgeID) + "\n");
		return false;
	}
	normalPath.lineTo(normal.pts[1].x, normal.pts[1].y);
	OpDebugImage::addArrowHeadToPath(normal, normalPath);
	OpDebugImage::drawPath(normalPath, color);
	return true;
}

bool OpDebugImage::drawTangent(OpVector tan, OpPoint midTPt, int id, uint32_t color) {
	LinePts tangent { midTPt, midTPt + tan };
	SkPath tangentPath;
	tangentPath.moveTo(tangent.pts[0].x, tangent.pts[0].y);
	if (!tangent.pts[1].isFinite()) {
		OpDebugOut("tangent not finite (id)" + STR(id) + "\n");
		return false;
	}
	tangentPath.lineTo(tangent.pts[1].x, tangent.pts[1].y);
	OpDebugImage::addArrowHeadToPath(tangent, tangentPath);
	OpDebugImage::drawPath(tangentPath, color);
	return true;
}

bool OpDebugImage::drawWinding(const OpCurve& curve, std::string left, std::string right,
		float normSign, uint32_t color) {
	for (bool allowIntersect : { false, true } ) {
		for (float normLength : { 4.f, 15.f } ) {
			for (float normT : { .58f, .38f, .78f, .18f, .98f } ) {
				OpVector norm = curve.normal(normT).normalize() * normLength;
				if (!norm.isFinite() || norm == OpVector{ 0, 0 })
					continue;
				OpPoint midTPt = curve.ptAtT(normT);
				SkRect bounds, rightBounds;
				std::string lefty = left + "_";
				labelFont.measureText(lefty.c_str(), lefty.length(), SkTextEncoding::kUTF8, 
						&bounds);
				labelFont.measureText(right.c_str(), right.length(), SkTextEncoding::kUTF8, 
						&rightBounds);
				float leftWidth = bounds.width();
				bounds.fRight += rightBounds.width();
				const int xOffset = 2;
				const int yOffset = 1;
				bounds.inset(-xOffset, -yOffset);
				SkRect trimmed = bounds;
				OpVector textOffset = norm * normSign;
				if (textOffset.dx < 0)
					textOffset.dx -= bounds.width();
				if (textOffset.dy > 0)
					textOffset.dy += bounds.height();
				OpPoint sumSide = midTPt + textOffset;
				trimmed.offset(sumSide.x, sumSide.y);
				if (!bitsAreBlank(allowIntersect, trimmed))
					continue;
				SkCanvas textLayer(bitmap);
				SkPaint paint;
				paint.setAntiAlias(true);
				paint.setColor(color);
				textLayer.drawString(SkString(left), sumSide.x, sumSide.y, labelFont, paint);
				paint.setColor(SK_ColorRED);
				textLayer.drawString(SkString(right), sumSide.x + leftWidth, sumSide.y,
						labelFont, paint);
				return true;
			}
		}
	}
	return false;
}

// fails may not have overflowed; they may have not found a place to draw the winding
bool OpDebugImage::drawEdgeWinding(const OpCurve& curve, const OpEdge* edge, uint32_t color) {
	bool success = true;
	const OpWinding& sum = edge->sum;
	OpContour* contour = edge->segment->contour;
	auto debugImageOut = contour->context->debugContextCallbacks.debugImageWindingOutFuncPtr;
	std::string sumLeft = debugImageOut && sum.isSet() ? (*debugImageOut)(sum.w, 0) : "?";
	std::string sumRight = debugImageOut && sum.isSet() ? (*debugImageOut)(sum.w, 1) : "?";
	if (!drawWinding(curve, sumLeft, sumRight, 1, color)) {
//		OpDebugOut("normalize overflowed: edge " + STR(edge->id) + "\n");
		success = false;
	}
	auto sumString = [edge, debugImageOut](const OpWinding& wind, const OpWinding& sum, int index) {
		if (!sum.isSet() && !wind.isSet())
			return std::string("?");
		OpContour* contour = edge->segment->contour;
		if (debugImageOut && !sum.isSet())
			return (*debugImageOut)(wind.w, index);
		OpWinding diffWind(edge->sum.w);
		contour->context->windingCallbacks.windingSubtractFuncPtr(diffWind.w, wind.w);
		return debugImageOut ? (*debugImageOut)(diffWind.w, index) : "";
	};
	std::string oppLeft = sumString(edge->winding, sum, 0);
	std::string oppRight = sumString(edge->winding, sum, 1);
	if (!drawWinding(curve, oppLeft, oppRight, -1, color)) {
//		OpDebugOut("normalize overflowed: edge " + STR(edge->id) + "\n");
		success = false;
	}
	return success;
}

bool OpDebugImage::drawCurve(OpCurve& curve, uint32_t color) {
	SkPath curvePath;
	OP_ASSERT(curve.debugIsLine());	// !!! add more types as needed
	curvePath.moveTo(curve.firstPt().x, curve.firstPt().y);
	curvePath.lineTo(curve.lastPt().x, curve.lastPt().y);
	OpDebugImage::drawPath(curvePath, color);
	return true;
}

#if 0
void OpDebugImage::drawLines() {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	for (OpDebugRay& line : lines) {
		if (!line.useAxis)
			continue;
		if (!drawValuesOn && !drawHexOn)
			continue;
		SkString label = SkString(STR(line.value));
		OpPoint mappedPt;
		if (Axis::vertical == line.axis) {
			mappedPt = DebugOpPtToPt(OpPoint(line.value, 0));
			offscreen.save();
			offscreen.rotate(-90, mappedPt.x, bitmapWH / 2);
			offscreen.drawString(label, mappedPt.x - 10, bitmapWH / 2, labelFont, paint);
			offscreen.restore();
		} else {
			mappedPt = DebugOpPtToPt(OpPoint(0, line.value));
			offscreen.drawString(label, bitmapWH / 2, mappedPt.y - 10, labelFont, paint);
		}
	}
}
#endif

void add(std::vector<OpEdge*>& e) {
	for (auto edge : e) {
		edge->debugDraw = true;
//        edge->debugOne = true;
	}
	OpDebugImage::focusEdges();
}

void add(std::vector<OpEdge>& e) {
	for (auto& edge : e) {
		edge.debugDraw = true;
//        edge.debugOne = true;
	}
	OpDebugImage::focusEdges();
}

#if OP_DEBUG_VERBOSE
void drawDepth(int level) {
	OpEdgeStorage* ccStorage = debugGlobalContext->ccStorage;
	if (!ccStorage)
		return;
	OpCurveCurve* cc = debugGlobalContext->debugCurveCurve;
	if (!cc)
		return;
	int count = ccStorage->debugCount();
	for (int index = 0; index < count; ++index) {
		OpEdge* edge = ccStorage->debugIndex(index);
        if (edge->debugDepth == level) {
		    edge->debugDraw = true;
//            edge->debugOne = true;
        } else
		    edge->debugDraw = false;
	}
	OpDebugImage::drawDoubleFocus();
}
#endif

void draw(std::vector<OpEdge*>& e) {
	hideEdges();
	add(e);
}

void draw(std::vector<OpEdge>& e) {
	hideEdges();
	add(e);
}

void draw(Axis axis, float value) {
	OpDebugImage::add(axis, value);
	drawLinesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const LinePts& ray) {
	OpDebugImage::add(ray);
	drawLinesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(float x1, float y1, float x2, float y2) {
    LinePts linePts {{{ {x1, y1}, {x2, y2} }}};
    draw(linePts);
}

void draw(const OpPoint& pt) {
	OpPtT ptT = { pt, OpNaN };
	draw(ptT);
}

void draw(const OpPtT& ptT) {
	OpDebugImage::add(ptT);
	drawPointsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const OpRect& r) {
	OpDebugImage::add(r);
	drawLinesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const PathOpsV0Lib::AddCurve& c) {
	OpDebugImage::add(c);
	drawCurvesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const PathOpsV0Lib::Curve& c) {
	OpDebugImage::add(c);
	drawCurvesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const OpCurve& curve) {
	draw(curve.c);
}

void draw(const OpPoint* pt) {
	draw(*pt);
}

void draw(const OpPtT* ptT) {
	draw(*ptT);
}

void draw(const OpRect* r) {
	draw(*r);
}

void draw(const PathOpsV0Lib::AddCurve* c) {
	draw(*c);
}

void draw(const PathOpsV0Lib::Curve* c) {
	draw(*c);
}

void draw(const OpCurve* curve) {
	draw(curve->c);
}

void draw(float x, float y) {
	draw(OpPoint(x, y));
}

void drawHex(uint32_t x, uint32_t y) {
	draw(OpDebugBitsToFloat(x), OpDebugBitsToFloat(y));
}

bool OpSegment::debugContains(const OpEdge* edge) const {
	for (auto& e : edges) {
		if (edge == &e)
			return true;
	}
	return false;
}

OpPoint OpDebugImage::find(int id, float t) {
	for (auto edge : edgeIterator) {
		if (id != edge->id)
			continue;
		edge->debugDraw = true;
//		edge->debugOne = true;
		drawIDsOn = true;
		return edge->curve.ptAtT(t);
	}
	const OpSegment* segment = nullptr;
	for (auto s : segmentIterator) {
		if (id == s->id)
			segment = s;
	}
	if (segment) {
		DRAW_IDS_ON(Segments);
		return segment->c.ptAtT(t);
	}
#if OP_DEBUG
	const OpIntersection* sect = nullptr;
	for (auto i : intersectionIterator) {
		if (id == i->id)
			sect = i;
	}
	if (sect) {
		DRAW_IDS_ON(Intersections);
		if (t != sect->ptT.t)
			OpDebugOut("intersection id " + STR(id) + " does not match t " + STR(t) + "\n");
		return sect->ptT.pt;
	}
#endif
#if OP_DEBUG
	auto coins = findCoincidence(id);
	if (coins.size()) {
		DRAW_IDS_ON(Coincidences);
		for (auto coin : coins) {
			if (t == coin->ptT.t)
				return coin->ptT.pt;
		}
		OpDebugOut("coincidence id " + STR(id) + " does not match t " + STR(t) + "\n");
		return OpPoint();
	}
#endif
	OpDebugOut("id " + STR(id) + " not found\n");
	return OpPoint();
}

// !!! macroize?
void drawT(int id, float t) {
	OpPoint pt = OpDebugImage::find(id, t);
	if (!pt.isFinite())
		return;
	draw(pt);
}

void drawT(int id, const OpPtT& ptT) {
	drawT(id, ptT.t);
}

void drawT(int id, const OpPtT* ptT) {
	drawT(id, *ptT);
}

void drawT(const OpCurve& curve, float t) {
	draw(curve.ptAtT(t));
}

void drawT(const OpCurve& curve, const OpPtT& ptT) {
	drawT(curve, ptT.t);
}

void drawT(const OpCurve& curve, const OpPtT* ptT) {
	drawT(curve, *ptT);
}

void drawT(const OpCurve* curve, float t) {
	drawT(*curve, t);
}

void drawT(const OpCurve* curve, const OpPtT& ptT) {
	drawT(*curve, ptT);
}

void drawT(const OpCurve* curve, const OpPtT* ptT) {
	drawT(*curve, ptT);
}

void drawT(const OpEdge& edge, float t) {
	draw(edge.curve.ptAtT(t));
}

void drawT(const OpEdge& edge, const OpPtT& ptT) {
	drawT(edge, ptT.t);
}

void drawT(const OpEdge& edge, const OpPtT* ptT) {
	drawT(edge, *ptT);
}

void drawT(const OpEdge* edge, float t) {
	drawT(*edge, t);
}

void drawT(const OpEdge* edge, const OpPtT& ptT) {
	drawT(*edge, ptT);
}

void drawT(const OpEdge* edge, const OpPtT* ptT) {
	drawT(*edge, ptT);
}

void drawT(const OpSegment& segment, float t) {
	draw(segment.c.ptAtT(t));
}

void drawT(const OpSegment& segment, const OpPtT& ptT) {
	drawT(segment, ptT.t);
}

void drawT(const OpSegment& segment, const OpPtT* ptT) {
	drawT(segment, *ptT);
}

void drawT(const OpSegment* segment, float t) {
	drawT(*segment, t);
}

void drawT(const OpSegment* segment, const OpPtT& ptT) {
	drawT(*segment, ptT);
}

void drawT(const OpSegment* segment, const OpPtT* ptT) {
	drawT(*segment, ptT);
}

void drawT(const PathOpsV0Lib::AddCurve& curve, float t) {
	OpCurve c(curve, Rotated::debug);
	draw(c.ptAtT(t));
}

void drawT(const PathOpsV0Lib::Curve& curve, float t) {
	OpCurve c(curve, Rotated::debug);
	draw(c.ptAtT(t));
}

void drawT(const PathOpsV0Lib::AddCurve& curve, const OpPtT& ptT) {
	drawT(curve, ptT.t);
}

void drawT(const PathOpsV0Lib::Curve& curve, const OpPtT& ptT) {
	drawT(curve, ptT.t);
}

void drawT(const PathOpsV0Lib::AddCurve& curve, const OpPtT* ptT) {
	drawT(curve, *ptT);
}

void drawT(const PathOpsV0Lib::Curve& curve, const OpPtT* ptT) {
	drawT(curve, *ptT);
}

void drawT(const PathOpsV0Lib::AddCurve* curve, float t) {
	drawT(*curve, t);
}

void drawT(const PathOpsV0Lib::Curve* curve, float t) {
	drawT(*curve, t);
}

void drawT(const PathOpsV0Lib::AddCurve* curve, const OpPtT& ptT) {
	drawT(*curve, ptT);
}

void drawT(const PathOpsV0Lib::Curve* curve, const OpPtT& ptT) {
	drawT(*curve, ptT);
}

void drawT(const PathOpsV0Lib::AddCurve* curve, const OpPtT* ptT) {
	drawT(*curve, ptT);
}

void drawT(const PathOpsV0Lib::Curve* curve, const OpPtT* ptT) {
	drawT(*curve, ptT);
}

void help() {
	OpDebugOut("navigation: l(eft) r(ight) u(p) d(own) i(n) oo(ut)\n");
	OpDebugOut("focus(id) resetFocus() center(id) clear()\n");
	OpDebugOut("showLeft() showRight() showPoints() showIDs()\n");
	OpDebugOut("hideIDs() hideValues() hideHex()\n");
}

#if 0
void resetFocus() {
	OpPointBounds focusRect;
	if (!drawSegmentsOn) {
		for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			focusRect.add(edge->bounds);
		}
	}
	if (!focusRect.isFinite()) {
		for (auto contour : contourIterator) {
			PathOpsV0Lib::DebugNativePath debugNativePath = contour->debugCallbacks.debugNativePathFuncPtr;
			if (!debugNativePath)
				continue;
			SkPath* path = (SkPath*) (*debugNativePath)(contour->debugContourData[
                    (size_t) PathOpsV0Lib::DebugContourType::windingUserData]);
			SkRect skrect = path->getBounds();
			focusRect.left = std::min(skrect.fLeft, focusRect.left);
			focusRect.top = std::min(skrect.fTop, focusRect.top);
			focusRect.right = std::max(skrect.fRight, focusRect.right);
			focusRect.bottom = std::max(skrect.fBottom, focusRect.bottom);
		}
	}
	if (focusRect.isFinite()) {
		OpDebugImage::drawDoubleFocus(focusRect, false);
		oo();
	} else
		OpDebugOut("operand bounds are not finite\n");
}
#endif

void u(float s) {
	DebugOpOffsetCenter(0, -DebugOpTranslate(s));
	OpDebugImage::drawDoubleFocus();
}

void u() {
	u(1);
}

void d(float s) {
	DebugOpOffsetCenter(0, +DebugOpTranslate(s));
	OpDebugImage::drawDoubleFocus();
}

void d() {
	d(1);
}

void l(float s) {
	DebugOpOffsetCenter(-DebugOpTranslate(s), 0);
	OpDebugImage::drawDoubleFocus();
}

void l() {
	l(1);
}

void r(float s) {
	DebugOpOffsetCenter(+DebugOpTranslate(s), 0);
	OpDebugImage::drawDoubleFocus();
}

void r() {
	r(1);
}

void i(float s) {
	DebugOpOffsetZoom(+s);
	OpDebugImage::drawDoubleFocus();
}

void i() {
	i(1);
}

void oo(float s) {
	DebugOpOffsetZoom(-s);
	OpDebugImage::drawDoubleFocus();
}

void oo() {
	oo(1);
}

#include "PathOpsTypes.h"
#include "curves/ConicBezier.h"

namespace PathOpsV0Lib {

void debugLineToSkPath(Curve c, SkPath& path) {
	path.lineTo(c.data->end.x, c.data->end.y);
}

void debugQuadToSkPath(Curve c, SkPath& path) {
    OpPoint controlPt = *(OpPoint*) CurveUserData(c.data);
	path.quadTo(controlPt.x, controlPt.y, c.data->end.x, c.data->end.y);
}

void debugConicToSkPath(Curve c, SkPath& path) {
    PointWeight control = *(PointWeight*) CurveUserData(c.data);
	path.conicTo(control.pt.x, control.pt.y, c.data->end.x, c.data->end.y, control.weight);
}

void debugCubicToSkPath(Curve c, SkPath& path) {
    OpPoint* controls = (OpPoint*) CurveUserData(c.data);
	path.cubicTo(controls[0].x, controls[0].y, controls[1].x, controls[1].y, 
            c.data->end.x, c.data->end.y);
}

}

#endif
