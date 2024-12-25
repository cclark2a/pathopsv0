// (c) 2023, Cary Clark cclark2@gmail.com
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
#include "OpContour.h"
#include "OpCurveCurve.h"
#include "OpEdge.h"
#include "OpJoiner.h"
#include "OpMath.h"
#include "OpSegment.h"
#include "OpTightBounds.h"
#include "OpWinder.h"
#include "PathOps.h"

SkBitmap bitmap;
SkFont labelFont(nullptr, 14, 1, 0);

std::vector<OpDebugRay> lines;
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

uint32_t OP_DEBUG_MULTICOLORED = 0xAbeBeBad;
uint32_t pathsOutColor = blue;

SkBitmap& bitmapRef() {
	return bitmap;
}

#define DRAW_IDS_ON(Thing) \
	do { \
		draw##Thing##On = true; \
		drawIDsOn = true; \
	} while (false)


struct OpDebugSegmentIter {
	OpDebugSegmentIter(bool start) {
		segmentIndex = 0;
		if (start)
			return;
		for (const auto c : debugGlobalContours->contours) {
			segmentIndex += c->segments.size();
		}
	}

	bool operator!=(OpDebugSegmentIter rhs) { 
		return segmentIndex != rhs.segmentIndex; 
	}

	OpSegment* operator*() {
		size_t index = 0;
		for (auto c : debugGlobalContours->contours) {
			for (auto& seg : c->segments) {
				if (index == segmentIndex)
					return &seg;
				++index;
			}
		}
		OpDebugOut("iterator out of bounds! segmentIndex: " + STR(segmentIndex) + "\n");
		return nullptr; 
	}

	void operator++() { 
		++segmentIndex;
	}

	size_t segmentIndex;
};

struct OpDebugSegmentIterator {
	OpDebugSegmentIter begin() { return OpDebugSegmentIter(true); }
	OpDebugSegmentIter end() { return OpDebugSegmentIter(false); }
	bool empty() { return !(begin() != end()); }
};

OpDebugSegmentIterator segmentIterator;

struct OpDebugEdgeIter {
	OpDebugEdgeIter(bool start)
		: isCurveCurve(false)
		, isFiller(false)
		, isLine(false) {
		edgeIndex = 0;
		if (start)
			return;
		for (const auto c : debugGlobalContours->contours) {
			for (const auto& s : c->segments)
				edgeIndex += (int) s.edges.size();
		}
		if (debugGlobalContours->fillerStorage)
			edgeIndex += debugGlobalContours->fillerStorage->debugCount();
		if (debugGlobalContours->ccStorage)
			edgeIndex += debugGlobalContours->ccStorage->debugCount();
	}

	bool operator!=(OpDebugEdgeIter rhs) { 
		return edgeIndex != rhs.edgeIndex; 
	}

	OpEdge* operator*() {
		int index = 0;
		for (auto c : debugGlobalContours->contours) {
			for (auto& s : c->segments) {
				for (auto& edge : s.edges) {
					if (index == edgeIndex) {
						isCurveCurve = false;
						isFiller = false;
						isLine = edge.curve.debugIsLine();
						return &edge;
					}
					++index;
				}
			}
		}
		if (debugGlobalContours->fillerStorage) {
			OpEdge* filler = debugGlobalContours->fillerStorage->debugIndex(edgeIndex - index);
			if (filler) {
				isCurveCurve = false;
				isFiller = true;
				isLine = true;
				return filler;
			}
			index += debugGlobalContours->fillerStorage->debugCount();
		}
		if (debugGlobalContours->ccStorage) {
			OpEdge* ccEdge = debugGlobalContours->ccStorage->debugIndex(edgeIndex - index);
			if (ccEdge) {
				isCurveCurve = true;
				isFiller = false;
				isLine = false;
				return ccEdge;
			}
			index += debugGlobalContours->ccStorage->debugCount();
		}
		OpDebugOut("iterator out of bounds! edgeIndex: " + STR(edgeIndex) + 
				"; max index: " + STR(index) + "\n");
		return nullptr; 
	}

	void operator++() { 
		++edgeIndex;
	}

	bool isCurveCurve;
	bool isFiller;
	bool isLine;
	int edgeIndex;
};

struct OpDebugEdgeIterator {
	OpDebugEdgeIter begin() { return OpDebugEdgeIter(true); }
	OpDebugEdgeIter end() { return OpDebugEdgeIter(false); }
	bool empty() { return !(begin() != end()); }
};

OpDebugEdgeIterator edgeIterator;

struct OpDebugIntersectionIter {
	OpDebugIntersectionIter(bool start) {
		localIntersectionIndex = 0;
		if (start)
			return;
		for (const auto c : debugGlobalContours->contours) {
			for (const auto& seg : c->segments) {
				localIntersectionIndex += seg.sects.i.size();
			}
		}
	}

	bool operator!=(OpDebugIntersectionIter rhs) { 
		return localIntersectionIndex != rhs.localIntersectionIndex; 
	}

	const OpIntersection* operator*() {
		size_t index = 0;
		for (const auto c : debugGlobalContours->contours) {
			for (const auto& seg : c->segments) {
				for (const auto sect : seg.sects.i) {
					if (index == localIntersectionIndex)
						return sect;
					++index;
				}
			}
		}
		OpDebugOut("iterator out of bounds! localIntersectionIndex: " + STR(localIntersectionIndex) + "\n");
		return nullptr; 
	}

	void operator++() { 
		++localIntersectionIndex;
	}

	size_t localIntersectionIndex;
};

struct OpDebugIntersectionIterator {
	OpDebugIntersectionIter begin() { return OpDebugIntersectionIter(true); }
	OpDebugIntersectionIter end() { return OpDebugIntersectionIter(false); }
	bool empty() { return !(begin() != end()); }
};

OpDebugIntersectionIterator intersectionIterator;


struct OpDebugDefeatDelete {
#if OP_DEBUG
	OpDebugDefeatDelete() {
		save = debugGlobalContours->debugInPathOps;
		debugGlobalContours->debugInPathOps = false;
	}
	~OpDebugDefeatDelete() {
		debugGlobalContours->debugInPathOps = save;
	}

	bool save;
#endif
};

void OpDebugImage::addToPath(const OpCurve& curve, SkPath& path) {
	path.moveTo(curve.firstPt().x, curve.firstPt().y);
	return curve.contours->debugCallBack(curve.c.type).addToPathFuncPtr(curve.c, path);
}

void OpDebugImage::init() {
	bitmap.allocPixels(SkImageInfo::MakeN32Premul(bitmapWH, bitmapWH));
	::clear();
	focusSegments();
}

// !!! missing pathsOutColor
void OpDebugImage::playback(FILE* file) {
//	FILE* file = fopen("OpDebugImageState.txt", "r");
	if (!file)
		return;
	std::vector<OpEdge*> ordered;
	for (auto edge : edgeIterator) {
		ordered.push_back(edge);
		edge->debugDraw = false;
		edge->debugCustom = 0;
	}
	std::sort(ordered.begin(), ordered.end(), [](const OpEdge* e1, const OpEdge* e2) {
			return e1->id < e2->id; });
	OpEdge* edge = ordered.front();
	const OpEdge* last = ordered.back();
	char str[255];
	double debugZoom;
	double debugCenter[2];
	float textSize;
	int intervals;
	int pPrecision, pSmall, pEpsilon;
	// required
	if (fscanf(file, "debugZoom: %lg\n", &debugZoom) != 1) {
		OpDebugOut("reading debugZoom failed\n");
		fclose(file);
		return;
	}
	DebugOpSetZoom(debugZoom);
	if (fscanf(file, "debugCenter: %lg, %lg\n", &debugCenter[0], &debugCenter[1]) != 2) {
		OpDebugOut("reading debugCenter failed\n");
		fclose(file);
		return;
	}
	DebugOpSetCenter(debugCenter[0], debugCenter[1]);
	DebugOpResetBounds();
	if (fscanf(file, "textSize: %g\n", &textSize) != 1) {
		OpDebugOut("reading textSize failed\n");
		fclose(file);
		return;
	}
	labelFont.setSize(textSize);
	if (fscanf(file, "gridIntervals: %d\n", &intervals) != 1) {
		OpDebugOut("reading gridIntervals failed\n");
		fclose(file);
		return;
	}
	gridIntervals = intervals;
	if (fscanf(file, "debugPrecision: %d\n", &pPrecision) != 1) {
		OpDebugOut("reading debugPrecision failed\n");
		fclose(file);
		return;
	}
	debugPrecision = pPrecision;
	if (fscanf(file, "debugSmall: %d\n", &pSmall) != 1) {
		OpDebugOut("reading debugSmall failed\n");
		fclose(file);
		return;
	}
	debugSmall = pSmall;
	if (fscanf(file, "debugEpsilon: %d\n", &pEpsilon) != 1) {
		OpDebugOut("reading debugEpsilon failed\n");
		fclose(file);
		return;
	}
	debugEpsilon = pEpsilon;
	// optional
	auto noMatch = [file](const char* str) {
		OpDebugOut("no match: " + std::string(str)); 
		fclose(file);
	};
	{
	#define OP_X(Thing) \
		draw##Thing##On = false;
		MASTER_LIST
		EDGE_BOOL_LIST
		ALIAS_LIST
		CALLOUT_LIST
	#undef OP_X
		while (fgets(str, sizeof(str), file)) {
	#define OP_X(Thing) \
			if (strlen(str) - 1 == strlen(#Thing) && 0 == strncmp(#Thing, str, strlen(#Thing))) \
				draw##Thing##On = true; \
			else
		MASTER_LIST
		EDGE_BOOL_LIST
		ALIAS_LIST
		CALLOUT_LIST
	#undef OP_X
			if (strlen(str) > strlen("edge: ")) {
				const char* idEdgeStr = str + strlen("edge: ");
				int id = strtol(idEdgeStr, nullptr, 0);
				while (edge != last && edge->id < id)
					++edge;
				if (edge->id == id) {  // ok if recorded edge does not exist
					edge->debugDraw = true;
					const char* colorStr = strstr(idEdgeStr, "color: ");
					if (colorStr)
						edge->debugCustom = strtoul(colorStr + strlen("color: "), nullptr, 0);
				} 	
			} else if (0 == strcmp("brief\n", str)) {
				break;
			} else
				return noMatch(str);
		}
	#define OP_X(Thing) \
		if (draw##Thing##On) \
			show##Thing();
		ALIAS_LIST
		CALLOUT_LIST
	#undef OP_X
		redraw();
	}
//	fclose(file);
}

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

void OpDebugImage::drawDoubleFocus() {
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
		for (auto contour : debugGlobalContours->contours) {
			if (contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller)) {
				SkPath* skPath = (SkPath*) contour->debugCallBacks.debugNativePathFuncPtr(contour->debugCaller);
				OP_ASSERT(skPath);
				drawDoubleFill(skPath->makeTransform(matrix), 
						first ? OpDebugAlphaColor(alpha, red) : OpDebugAlphaColor(alpha, blue));
			}
			first = false;
		}
		if (drawResultOn && debugGlobalContours->callerOutput)
			drawDoubleFill(((SkPath*) debugGlobalContours->callerOutput)
					->makeTransform(matrix), OpDebugAlphaColor(alpha, green));
	}
	if (drawLinesOn)
		DebugOpDraw(lines);
	// set up default colors for all edges
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) {
		OpEdge* edge = *edgeIter;
		if (edge->disabled)
			edge->debugColor = red;
		else if (edge->inOutput)
			edge->debugColor = orange;
		else if (Unsortable::none != edge->isUnsortable)
			edge->debugColor = purple;
		else if (edgeIter.isCurveCurve) {
			if (edge->ccOverlaps)
				edge->debugColor = edge->winding.contour->debugCallBacks
						.debugIsOppFuncPtr(edge->winding.contour->debugCaller)
						? orange : darkGreen;
			else
				edge->debugColor = purple;
		} else
			edge->debugColor = black;
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
	if (drawIDsOn || drawNormalsOn || drawTangentsOn
			|| drawWindingsOn || drawEndToEndOn || drawControlLinesOn) {
		for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			if (ids.end() != std::find(ids.begin(), ids.end(), edge->id))
				continue;
			ids.push_back(edge->id);
			uint32_t color = edge->debugCustom ? edge->debugCustom : edge->debugColor;
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
	if (drawSegmentsOn && drawTangentsOn) {
		for (auto segment : segmentIterator)
			DebugOpDrawSegmentTangent(segment, segment->debugColor);
	}
	if (drawGridOn)
		drawGrid();
	if (drawRasterOn)
		drawRaster();
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
	std::vector<OpEdge*> ordered;
	for (auto edge : edgeIterator) {
		ordered.push_back(edge);
		edge->debugDraw = false;
	}
	std::sort(ordered.begin(), ordered.end(), [](const OpEdge* e1, const OpEdge* e2) {
			return e1->id < e2->id; });
	for (OpEdge* e : ordered) {
		if (!e->debugDraw)
			continue;
		fprintf(recordFile, "edge: %d", e->id);
		if (e->debugCustom)
			fprintf(recordFile, " color: 0x%08x", e->debugCustom);
		fprintf(recordFile, "\n");
	}
//	fclose(recordFile);
}

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

bool OpDebugImage::find(int id, OpPointBounds* boundsPtr, OpPoint* pointPtr) {
	if (std::vector<const OpIntersection*> coins = findCoincidence(id); coins.size()) {
		DRAW_IDS_ON(Coincidences);
		// !!! wrong: add rect formed by both intersections with this id
		for (auto coin : coins)
			boundsPtr->add(coin->ptT.pt);
		return true;
	}
	if (OpEdge* edge = const_cast<OpEdge*>(findEdge(id))) {
		edge->debugDraw = true;
		drawIDsOn = true;
		*boundsPtr = edge->ptBounds;
		return true;
	}
	if (std::vector<const OpEdge*> outputs = findEdgeOutput(id); outputs.size()) {
		drawIDsOn = true;
		for (auto output : outputs)
			boundsPtr->add(output->ptBounds);
		return true;
	}
	if (std::vector<const OpEdge*> matches = findEdgeRayMatch(id); matches.size()) {
		drawIDsOn = true;
		for (auto match : matches)
			boundsPtr->add(match->ptBounds);
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
		drawIDsOn = true;
		*boundsPtr = limb->edge->ptBounds;
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

#if 0
// !!! not sure I need this; but it does raise the question if dump and image need their own finds
std::vector<const OpEdge*> OpDebugImage::find(int id) {
	extern OpEdge* findEdge(int id);
	extern std::vector<const OpEdge*> findEdgeOutput(int id);
	std::vector<const OpEdge*> result;
	if (OpEdge* edge = findEdge(id))
		result.push_back(edge);
	if (std::vector<const OpEdge*> oEdges = findEdgeOutput(id); oEdges.size())
		result.insert(result.end(), oEdges.begin(), oEdges.end());
	return result;
}
#endif

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

void addFocus(const OpContour& contour) {
	for (const OpSegment& s : contour.segments)
		addFocus(s.ptBounds);
}

void addFocus(OpContours& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		addFocus(contour);
	addFocus(bounds);
}

// !!! remove all const everywhere
void addFocus(const OpContours& contours) {
	addFocus(*const_cast<OpContours*>(&contours));
}

void addFocus(const OpEdge& edge) {
	addFocus(edge.ptBounds);
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

void addFocus(const OpContour* contour) {
	addFocus(*contour);
}

void addFocus(const OpContours* contours) {
	addFocus(*contours);
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

void ctr() {
	ctr(*debugGlobalContours);
}

void ctr(int id) {
	OpDebugImage::center(id, false);
}

void ctr(float x, float y) {
	ctr(OpPoint(x, y));
}

void ctr(const OpContour& contour) {
	OpPointBounds bounds;
	for (auto& segment : contour.segments)
		bounds.add(segment.ptBounds);
	ctr(bounds);
}

void ctr(OpContours& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		for (auto& segment : contour->segments)
			bounds.add(segment.ptBounds);
	ctr(bounds);
}

void ctr(const OpContours& contours) {
	ctr(*const_cast<OpContours*>(&contours));
}

void ctr(const OpEdge& edge) {
	ctr(edge.ptBounds);
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

void ctr(const OpContour* contour) {
	ctr(*contour);
}

void ctr(const OpContours* contours) {
	ctr(*contours);
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

void focus(int id) {
	OpDebugImage::focus(id, false);
}

void focus(const OpContour& contour) {
	drawSegmentsOn = true;
	OpPointBounds bounds;
	for (auto& segment : contour.segments)
		bounds.add(segment.ptBounds);
	focus(bounds);
}

void focus(OpContours& contours) {
	OpPointBounds bounds;
	for (auto contour : contours.contours)
		for (auto& segment : contour->segments)
			bounds.add(segment.ptBounds);
	focus(bounds);
}

// !!! remove all const everywhere
void focus(const OpContours& contours) {
	focus(*const_cast<OpContours*>(&contours));
}

void focus(const OpEdge& edge) {
	focus(edge.ptBounds);
}

void focus(const OpRect& rect) {
	OpDebugImage::drawDoubleFocus(rect, false);
}

void focus(const OpSegment& segment) {
	drawSegmentsOn = true;
	focus(segment.ptBounds);
}

void focus(const OpContour* contour) {
	focus(*contour);
}

void focus(const OpContours* contours) {
	focus(*contours);
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

void OpDebugImage::focusEdges() {
	if (edgeIterator.empty())
		return;
	OpPointBounds focusRect;
	for (auto edge : edgeIterator) {
		if (!edge->debugDraw)
			continue;
		focusRect.add(edge->ptBounds);
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
	OpDebugImage::clearLines();
	DebugOpResetFocus();
}

void clearLines() {
	OpDebugImage::clearLines();
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
	OpPointBounds bounds = debugGlobalContours->maxBounds;
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
	SkPath lPath = draw(debugGlobalContours->debugData.leftRaster);
	OpDebugImage::drawDoubleFill(lPath, 0x7fff0000);
	SkPath rPath = draw(debugGlobalContours->debugData.rightRaster);
	OpDebugImage::drawDoubleFill(rPath, 0x7f00ff00);
	SkPath cPath = draw(debugGlobalContours->debugData.combinedRaster);
	OpDebugImage::drawDoubleFill(cPath, 0x7f0000ff);
	SkPath oPath = draw(&debugGlobalContours->opRaster);
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
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY));
				DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY));
				break;
			case SkPath::kQuad_Verb:
			case SkPath::kConic_Verb:
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY));
				if (drawControlsOn)
					DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY));
				DebugOpBuild(OpPoint(pts[2].fX, pts[2].fY));
				break;
			case SkPath::kCubic_Verb:
				DebugOpBuild(OpPoint(pts[0].fX, pts[0].fY));
				if (drawControlsOn) {
					DebugOpBuild(OpPoint(pts[1].fX, pts[1].fY));
					DebugOpBuild(OpPoint(pts[2].fX, pts[2].fY));
				}
				DebugOpBuild(OpPoint(pts[3].fX, pts[3].fY));
				break;
			case SkPath::kClose_Verb:
			case SkPath::kDone_Verb:
				break;
			}
		} while (verb != SkPath::kDone_Verb);
	};
	for (auto contour : debugGlobalContours->contours) {
		if (contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller)) {
			SkPath* skPath = (SkPath*) contour->debugCallBacks.debugNativePathFuncPtr(contour->debugCaller);
			drawPathPt(skPath);
		}
	}
	if (drawSegmentsOn) {
		for (auto seg : segmentIterator) {
			DebugOpBuild(seg->c.firstPt());
			DebugOpBuild(seg->c.lastPt());
			// !!! probably need switch to say 'draw control points'
			if (drawControlsOn) {
				for (int index = 1; index < seg->c.pointCount() - 1; ++index)
					DebugOpBuild(seg->c.hullPt(index));
			}
		}
	}
	for (auto edge : edgeIterator) {
		if (!edge->debugDraw)
			continue;
		bool isOpp = edge->winding.contour ? edge->winding.contour->debugCallBacks
				.debugIsOppFuncPtr(edge->winding.contour->debugCaller) : false;
		DebugOpBuild(edge->startPt(), edge->startT, isOpp);
		DebugOpBuild(edge->endPt(), edge->endT, isOpp);
		if (drawControlsOn) {
			for (int index = 1; index < edge->curve.pointCount() - 1; ++index)
				DebugOpBuild(edge->curve.hullPt(index));
		}
		if (drawCentersOn)
			DebugOpBuild(edge->center.pt, edge->center.t, DebugSprite::square);
		if (drawHullsOn) {
			for (const HullSect& hull : edge->hulls.h)
				DebugOpBuild(hull.sect.pt, hull.sect.t, DebugSprite::circle);
		}
		if (drawEdgeRunsOn && debugGlobalContours->debugCurveCurve) {

		}
	}
	if (drawIntersectionsOn) {
		for (const auto& sect : intersectionIterator) {
			DebugOpBuild(sect->ptT.pt);
		}
	}
	if (drawLinesOn) {
		for (const auto& line : lines) {
			for (auto contour : debugGlobalContours->contours) {
				if (contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller))
					DebugOpBuild(*(SkPath*)contour->debugCallBacks.debugNativePathFuncPtr(contour->debugCaller), line);
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
		DebugOpBuild(ptT.pt, ptT.t, DebugSprite::triangle);
	if (drawValuesOn) {
		if (drawTsOn)
			DebugOpDrawT(drawHexOn);
		else
			DebugOpDrawValue(drawHexOn);
	}
	DebugOpDrawSprites();
}

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

void OpDebugImage::clearIntersections() {
	drawIntersectionsOn = false;
}

void OpDebugImage::clearLines() {
	lines.clear();
	drawLinesOn = false;
}

void OpDebugImage::clearPoints() {
	ptTs.clear();
	drawPointsOn = false;
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

void hideLeft() {
	OpContour* contour = debugGlobalContours->contours.front();
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, false);
	OpDebugImage::drawDoubleFocus();
	drawLeftOn = false;
}

void showLeft() {
	OpContour* contour = debugGlobalContours->contours.front();
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, true);
	OpDebugImage::drawDoubleFocus();
	drawLeftOn = true;
}

void toggleLeft() {
	OpContour* contour = debugGlobalContours->contours.front();
	bool debugDraw = contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller);
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, !debugDraw);
	OpDebugImage::drawDoubleFocus();
	drawLeftOn ^= true;
}

void hideRight() {
	OpContour* contour = debugGlobalContours->contours.back();
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, false);
	OpDebugImage::drawDoubleFocus();
	drawRightOn = false;
}

void showRight() {
	OpContour* contour = debugGlobalContours->contours.back();
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, true);
	OpDebugImage::drawDoubleFocus();
	drawRightOn = true;
}

void toggleRight() {
	OpContour* contour = debugGlobalContours->contours.back();
	bool debugDraw = contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller);
	contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, !debugDraw);
	OpDebugImage::drawDoubleFocus();
	drawRightOn ^= true;
}

static void operateOnLimbEdges(std::function<void (OpEdge*)> fun) {
	const OpTree* tree = debugGlobalContours->debugTree;
	if (!tree)
		return;
	for (int index = 0; index < tree->totalUsed; ++index) {
		const OpLimb& limb = debugGlobalContours->debugNthLimb(index);
		OpEdge* edge = limb.edge;
		bool callFun = 0 >= limbsShown || limbsShown > index;
		if (callFun)
			fun(edge);
		else
			edge->debugDraw = false;
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

void hideOperands() {
	for (auto contour : debugGlobalContours->contours) {
		contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, false);
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn = false;
}

void showOperands() {
	for (auto contour : debugGlobalContours->contours) {
		contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, true);
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn = true;
}

void toggleOperands() {
	for (auto contour : debugGlobalContours->contours) {
		bool debugDraw = contour->debugCallBacks.debugGetDrawFuncPtr(contour->debugCaller);
		contour->debugCallBacks.debugSetDrawFuncPtr(contour->debugCaller, !debugDraw);
	}
	OpDebugImage::drawDoubleFocus();
	drawOperandsOn ^= true;
}

static void operateOnEdgeRuns(std::function<void (OpEdge*)> fun) {
	const OpCurveCurve* cc = debugGlobalContours->debugCurveCurve;
	if (!cc)
		return;
	for (const CcCurves& curves : { cc->edgeCurves, cc->oppCurves } ) {
		for (const EdgeRun run : curves.runs) {
			OpEdge* edge = const_cast<OpEdge*>(run.edge);
			fun(edge);
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void hideEdgeRuns() {
	operateOnEdgeRuns([](OpEdge* edge) {
		edge->debugDraw = false;
	});
}

void showEdgeRuns() {
	operateOnEdgeRuns([](OpEdge* edge) {
		edge->debugDraw = true;
	});
}

void toggleEdgeRuns() {
	operateOnEdgeRuns([](OpEdge* edge) {
		edge->debugDraw ^= true;
	});
}

#define OP_X(Thing, edgeCheck) \
void color##Thing(uint32_t color) { \
	for (auto edge : edgeIterator) { \
		if (edgeCheck) { \
			edge->debugCustom = color; \
			edge->debugDraw = true; \
		} \
	} \
	OpDebugImage::drawDoubleFocus(); \
}
COLOR_LIST2
#undef OP_X

void colorLimbs(uint32_t color) {
	const OpTree* tree = debugGlobalContours->debugTree;
	if (!tree)
		return;
	for (int index = 0; index < tree->totalUsed; ++index) {
		const OpLimb& limb = debugGlobalContours->nthLimb(index);
		OpEdge* edge = limb.edge;
		edge->debugCustom = color;
		edge->debugDraw = true;
	}
}

void colorSegments(uint32_t color) {
	for (auto seg : segmentIterator) {
		seg->debugColor = color;
	}
	OpDebugImage::drawDoubleFocus();
}

extern std::vector<std::pair<uint32_t, std::string>> debugColorArray;

void colorOut(uint32_t color) {
	for (auto edge : edgeIterator) {
		if (!edge->inOutput)
			continue;
		if (OP_DEBUG_MULTICOLORED != color)
			edge->debugCustom = color;
		else
			edge->debugCustom = debugColorArray[edge->debugOutPath % debugColorArray.size()].first;
		edge->debugDraw = true;
	}
	OpDebugImage::drawDoubleFocus();
}

void colorOpp(uint32_t color) {
	for (auto edge : edgeIterator) {
		OpContour* contour = edge->winding.contour;
		if (contour->debugCallBacks.debugIsOppFuncPtr(contour->debugCaller)) {
			edge->debugCustom = color;
			edge->debugDraw = true;
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void colorPathsOut(uint32_t color) {
	pathsOutColor = color;
}

#define OP_X(Thing) \
void color##Thing() { \
	color##Thing(OP_DEBUG_MULTICOLORED); \
} \
void color##Thing(uint8_t alpha, uint32_t color) { \
	color##Thing((alpha << 24) | (color & 0x00FFFFFF)); \
} \
void uncolor##Thing() { \
	color##Thing(0); \
}
COLOR_LIST
#undef OP_X

void color(int id) {
	OpEdge* edge = findEdge(id);
	if (edge) {
		edge->debugCustom = OP_DEBUG_MULTICOLORED;
		edge->debugDraw = true;
	}
	OpDebugImage::drawDoubleFocus();
}

void color(int id, uint32_t c) {
	OpEdge* edge = findEdge(id);
	if (edge) {
		edge->debugCustom = c;
		edge->debugDraw = true;
	}
	OpDebugImage::drawDoubleFocus();
}

void uncolor(int id) {
	OpEdge* edge = findEdge(id);
	if (edge) {
		edge->debugCustom = 0;
		edge->debugDraw = true;
	}
	OpDebugImage::drawDoubleFocus();
}

void colorLink(OpEdge* edge, uint32_t color) {
	auto colorChain = [edge, color](EdgeMatch which) {
		const OpEdge* looped = edge->debugIsLoop(which, LeadingLoop::in);
		bool firstLoop = false;
		int safetyCount = 0;
		OpEdge* chain = edge;
		for (;;) {
			chain->debugCustom = color;
			chain->debugDraw = true;
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

void OpContours::debugLimbClear() {
	for (auto contour : contours) {
		for (auto& segment : contour->segments) {
			for (OpEdge& edge : segment.edges) {
				edge.debugLimb = false;
			}
		}
	}
}

int OpContours::debugLimbIndex(const OpEdge* edge) const {
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

void OpEdge::color(uint32_t c) {
	debugCustom = c;
	debugDraw = true;
	OpDebugImage::drawDoubleFocus();
}

void OpEdge::addLink() {
	OpEdge* chain = this;
	std::vector<OpEdge*> seen;
	do {
		chain->debugDraw = true;
		seen.push_back(chain);
	} while ((chain = chain->nextEdge) && seen.end() == std::find(seen.begin(), seen.end(), chain));
	chain = this;
	while ((chain = chain->priorEdge) && seen.end() == std::find(seen.begin(), seen.end(), chain)) {
		chain->debugDraw = true;
		seen.push_back(chain);
	}
	drawIDsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void OpEdge::drawLink() {
	hideEdges();
	addLink();
}

void OpJoiner::debugDraw() {
	for (auto e : byArea)
		e->debugDraw = true;
	for (auto e : unsectByArea)
		e->debugDraw = true;
	OpDebugImage::focusEdges();
}

void OpTree::debugLimbEdges(OpEdge* edge) {
	OP_ASSERT(!edge->debugIsLoop());
	if (edge->priorEdge)
		edge = const_cast<OpEdge*>(edge->debugAdvanceToEnd(EdgeMatch::start));
	OP_ASSERT(edge->lastEdge || !edge->nextEdge);
	do {
		edge->debugLimb = true;
	} while ((edge = edge->nextEdge));
}

void OpWinder::debugDraw() {
	for (auto e : inX)
		e->debugDraw = true;
	for (auto e : inY)
		e->debugDraw = true;
	OpDebugImage::focusEdges();
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
	std::string sumLeft = sum.isSet() ? sum.contour->debugCallBacks.debugImageOutFuncPtr(sum.w, 0) : "?";
	std::string sumRight = sum.isSet() ? sum.contour->debugCallBacks.debugImageOutFuncPtr(sum.w, 1) : "?";
	if (!drawWinding(curve, sumLeft, sumRight, 1, color)) {
//		OpDebugOut("normalize overflowed: edge " + STR(edge->id) + "\n");
		success = false;
	}
	auto sumString = [edge](const OpWinding& wind, const OpWinding& sum, int index) {
		if (!sum.isSet() && !wind.isSet())
			return std::string("?");
		if (!sum.isSet())
			return wind.contour->debugCallBacks.debugImageOutFuncPtr(wind.w, index);
		OpWinding diffWind(edge->sum.contour, edge->sum.w);
		wind.contour->callBacks.windingSubtractFuncPtr(diffWind.w, wind.w);
		return wind.contour->debugCallBacks.debugImageOutFuncPtr(diffWind.w, index);
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
	}
	OpDebugImage::focusEdges();
}

void add(std::vector<OpEdge>& e) {
	for (auto& edge : e) {
		edge.debugDraw = true;
	}
	OpDebugImage::focusEdges();
}

#if OP_DEBUG_VERBOSE
void drawDepth(int level) {
	OpEdgeStorage* ccStorage = debugGlobalContours->ccStorage;
	if (!ccStorage)
		return;
	int count = ccStorage->debugCount();
	for (int index = 0; index < count; ++index) {
		OpEdge* edge = ccStorage->debugIndex(index);
		edge->debugDraw = false;
	}
	OpCurveCurve* cc = debugGlobalContours->debugCurveCurve;
	if (!cc)
		return;
	if (level > 0) {
		size_t dvLevel = std::min((size_t) level, cc->dvDepthIndex.size() + 1);
		size_t lo = cc->dvDepthIndex[dvLevel - 1];
		size_t hi = cc->dvDepthIndex.size() <= dvLevel ? cc->dvAll.size() : cc->dvDepthIndex[dvLevel];
		if (lo >= hi)
			return;
		while (lo < hi) {
			cc->dvAll[lo]->debugDraw = true;
			++lo;
		}
	} else {
		for (const CcCurves& ccCurves : { cc->edgeCurves, cc->oppCurves } ) {
			for (OpEdge* edge : ccCurves.c) {
				edge->debugDraw = true;
			}
		}
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

void draw(const OpPoint& pt) {
	OpPtT ptT = { pt, OpNaN };
	draw(ptT);
}

void draw(const OpPtT& ptT) {
	OpDebugImage::add(ptT);
	drawPointsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw(const OpPoint* pt) {
	draw(*pt);
}

void draw(const OpPtT* ptT) {
	draw(*ptT);
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

void help() {
	OpDebugOut("navigation: l(eft) r(ight) u(p) d(own) i(n) oo(ut)\n");
	OpDebugOut("focus(id) resetFocus() center(id) clear()\n");
	OpDebugOut("showLeft() showRight() showPoints() showIDs()\n");
	OpDebugOut("hideIDs() hideValues() hideHex()\n");
}

void resetFocus() {
	OpPointBounds focusRect;
	if (!drawSegmentsOn) {
		for (auto edge : edgeIterator) {
			if (!edge->debugDraw)
				continue;
			focusRect.add(edge->ptBounds);
		}
	}
	if (!focusRect.isFinite()) {
		for (auto contour : debugGlobalContours->contours) {
			SkPath* path = (SkPath*) contour->debugCallBacks.debugNativePathFuncPtr(contour->debugCaller);
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

#endif
