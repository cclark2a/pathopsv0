#include "OpDebug.h"
#include "OpDebugImage.h"

#if OP_DEBUG_IMAGE
#pragma optimize( "", off )

#include <algorithm>
#include "include/core/SkBitmap.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkFont.h"
#include "include/core/SkImageInfo.h"
#include "include/core/SkPath.h"
#include "include/core/SkPaint.h"
#include "OpDebugDouble.h"
#include "OpDebugDump.h"
#include "OpContour.h"
#include "OpEdge.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpMath.h"
#include "OpSegment.h"
#include "OpTightBounds.h"
#include "PathOps.h"

SkBitmap bitmap;
SkFont labelFont(nullptr, 20, 1, 0);

std::vector<const OpIntersection*> coincidences;
std::vector<const OpEdge*> segmentEdges;
std::vector<const OpEdge*> temporaryEdges;
std::vector<const OpIntersection*> intersections;
std::vector<OpRay> lines;
std::vector<OpInPath> operands;
std::vector<const SkPath*> paths;
std::vector<const OpSegment*> segments;
int gridIntervals = 8;
int valuePrecision = -1;		// minus one means unset?

#define DRAW_FEATURE_GLOBAL(Thing) \
bool draw##Thing##On = false

DRAW_FEATURE_GLOBAL(Arrows);
DRAW_FEATURE_GLOBAL(Bounds);
DRAW_FEATURE_GLOBAL(Centers);
DRAW_FEATURE_GLOBAL(Chains);
DRAW_FEATURE_GLOBAL(Coincidences);
DRAW_FEATURE_GLOBAL(Controls);
DRAW_FEATURE_GLOBAL(Edges);
DRAW_FEATURE_GLOBAL(Grid);
DRAW_FEATURE_GLOBAL(Hex);
DRAW_FEATURE_GLOBAL(IDs);
DRAW_FEATURE_GLOBAL(Intersections);
DRAW_FEATURE_GLOBAL(Left);
DRAW_FEATURE_GLOBAL(Lines);
DRAW_FEATURE_GLOBAL(Normals);
DRAW_FEATURE_GLOBAL(Operands);
DRAW_FEATURE_GLOBAL(Paths);
DRAW_FEATURE_GLOBAL(Points);
DRAW_FEATURE_GLOBAL(Right);
DRAW_FEATURE_GLOBAL(SegmentEdges);
DRAW_FEATURE_GLOBAL(Segments);
DRAW_FEATURE_GLOBAL(Sums);
DRAW_FEATURE_GLOBAL(Tangents);
DRAW_FEATURE_GLOBAL(TemporaryEdges);
DRAW_FEATURE_GLOBAL(Ts);
DRAW_FEATURE_GLOBAL(Values);
DRAW_FEATURE_GLOBAL(Windings);

#undef DRAW_FEATURE_GLOBAL

#define DRAW_IDS_ON(Thing) \
	do { \
		draw##Thing##On = true; \
		drawIDsOn = true; \
	} while (false)


void OpDebugImage::addToPath(const OpCurve& curve, SkPath& path) {
	path.moveTo(curve.pts[0].x, curve.pts[0].y);
	switch (curve.type) {
		case noType:
			assert(0);
			break;
		case pointType:
			return;
		case lineType:
			path.lineTo(curve.pts[1].x, curve.pts[1].y);
			break;
		case quadType:
			path.quadTo(curve.pts[1].x, curve.pts[1].y, curve.pts[2].x, curve.pts[2].y);
			break;
		case conicType:
			path.conicTo(curve.pts[1].x, curve.pts[1].y, curve.pts[2].x, curve.pts[2].y, curve.weight);
			break;
		case cubicType:
			path.cubicTo(curve.pts[1].x, curve.pts[1].y, curve.pts[2].x, curve.pts[2].y,
					curve.pts[3].x, curve.pts[3].y);
			break;
		default:
			assert(0);
	}
}

void OpDebugImage::init(const OpInPath& left, const OpInPath& right) {
	bitmap.allocPixels(SkImageInfo::MakeN32Premul(bitmapWH, bitmapWH));
	::clear();
	operands.clear();
	paths.clear();
	operands.push_back(left);
	operands.push_back(right);
	paths.push_back(left.skPath);
	paths.push_back(right.skPath);
	drawLeftOn = true;
	drawRightOn = true;
	drawPathsOn = true;
	SkRect opBounds = left.skPath->getBounds();
	opBounds.join(right.skPath->getBounds());
	DebugOpSetBounds(opBounds.fLeft, opBounds.fTop, opBounds.fRight, opBounds.fBottom);
}

void OpDebugImage::clearScreen() {
	SkCanvas offscreen(bitmap);
	offscreen.clear(SK_ColorWHITE);
}

void OpDebugImage::drawPath(const SkPath& path) {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setStyle(SkPaint::kStroke_Style);
	offscreen.drawPath(path, paint);
}

void OpDebugImage::drawDoubleFocus() {
	std::vector<int> ids;
	clearScreen();
	if (drawPathsOn)
		DebugOpDraw(paths);
	if (drawLinesOn)
		DebugOpDraw(lines);
	if (drawSegmentsOn)
		DebugOpDraw(segments);
	if (drawEdgesOn) {
		DebugOpClearEdges();
		if (drawSegmentEdgesOn)
			DebugOpDraw(segmentEdges);
		if (drawTemporaryEdgesOn) {
			DebugOpDraw(temporaryEdges);
			if (OpEdgeIntersect::debugActive) {
				DebugOpDraw(OpEdgeIntersect::debugActive->edgeParts);
				DebugOpDraw(OpEdgeIntersect::debugActive->oppParts);
			}
		}
	}
	if (drawNormalsOn) {
		if (drawSegmentEdgesOn)
			OpDebugImage::drawEdgeNormals(segmentEdges);
		if (drawTemporaryEdgesOn) {
			OpDebugImage::drawEdgeNormals(temporaryEdges);
			if (OpEdgeIntersect::debugActive) {
				OpDebugImage::drawEdgeNormals(OpEdgeIntersect::debugActive->edgeParts);
				OpDebugImage::drawEdgeNormals(OpEdgeIntersect::debugActive->oppParts);
			}
		}
	}
	if (drawWindingsOn) {
		if (drawSegmentEdgesOn)
			OpDebugImage::drawEdgeWindings(segmentEdges);
		if (drawTemporaryEdgesOn) {
			OpDebugImage::drawEdgeWindings(temporaryEdges);
			if (OpEdgeIntersect::debugActive) {
				OpDebugImage::drawEdgeWindings(OpEdgeIntersect::debugActive->edgeParts);
				OpDebugImage::drawEdgeWindings(OpEdgeIntersect::debugActive->oppParts);
			}
		}
	}
	if (drawPointsOn)
		OpDebugImage::drawPoints();
	if (drawSegmentsOn && drawIDsOn)
		DebugOpDrawSegmentIDs(segments, ids);
	if (drawEdgesOn && drawIDsOn) {
		if (drawSegmentEdgesOn)
			DebugOpDrawEdgeIDs(segmentEdges, ids);
		if (drawTemporaryEdgesOn) {
			DebugOpDrawEdgeIDs(temporaryEdges, ids);
			if (OpEdgeIntersect::debugActive) {
				DebugOpDrawEdgeIDs(OpEdgeIntersect::debugActive->edgeParts, ids, false);
				DebugOpDrawEdgeIDs(OpEdgeIntersect::debugActive->oppParts, ids, true);
			}
		}
	}
	if (drawIntersectionsOn && drawIDsOn)
		DebugOpDrawIntersectionIDs(intersections, ids);
	if (drawSegmentsOn && drawCoincidencesOn)
		DebugOpDrawPointIDs(segments, ids);
	if (drawGridOn)
		drawGrid();
}

void OpDebugImage::drawDoubleCenter(OpPoint pt, bool add) {
	add ? DebugOpAddBounds(pt.x, pt.y, pt.x, pt.y) :	// !!! should suppress zoom change
			DebugOpSetCenter(pt.x, pt.y);
	drawDoubleFocus();
}

void OpDebugImage::drawDoubleFocus(const OpPointBounds& b, bool add) {
	add ? DebugOpAddBounds(b.left, b.top, b.right, b.bottom) :
			DebugOpSetBounds(b.left, b.top, b.right, b.bottom);
	drawDoubleFocus();
}

void OpDebugImage::drawDoublePath(const SkPath& path, uint32_t color) {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	if (SK_ColorBLACK != color) {
		paint.setColor(color);
		offscreen.drawPath(path, paint);
	}
	paint.setStyle(SkPaint::kStroke_Style);
	paint.setColor(SK_ColorBLACK);
	offscreen.drawPath(path, paint);
}

void OpDebugImage::drawGrid() {
	SkCanvas offscreen(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(0x3f000000);
	SkPaint textPaint = paint;
	paint.setStyle(SkPaint::kStroke_Style);
	const float wh = (float) bitmapWH;
 	const float interval = wh / gridIntervals;
	const int xOffset = 2;
	double left, top, right, bottom;
	DebugOpBounds(left, top, right, bottom);
	float xVal = left;
	float yVal = top;
	for (float x = 0; x < wh; x += interval) {
		if (x != 0) {
			offscreen.drawLine(x, 0, x, wh, paint);
			offscreen.drawLine(0, x, wh, x, paint);
		}
		if (!drawValuesOn)
			continue;
		std::string xValStr = drawHexOn ? OpDebugDumpHex(xVal) : OpDebugToString(xVal, valuePrecision);
		offscreen.drawString(SkString(xValStr), x + xOffset, bitmapWH - xOffset, labelFont, textPaint);
		xVal += (right - left) / gridIntervals;
		offscreen.save();
		offscreen.rotate(-90, 15, x - xOffset);
		std::string yValStr = drawHexOn ? OpDebugDumpHex(yVal) : OpDebugToString(yVal, valuePrecision);
		offscreen.drawString(SkString(yValStr), 15, x - xOffset, labelFont, textPaint);
		offscreen.restore();
		yVal += (bottom - top) / gridIntervals;
	}
}

// !!! should be named 'move grid center to' ? or should it keep center while zooming?
void gridCenter(int x, int y) {
	if (x < 0 || x >= gridIntervals || y < 0 || y >= gridIntervals)
		return OpDebugOut("parameters must be in grid interval range: 0 to " + STR(gridIntervals) + "\n");
	double left, top, right, bottom;
	DebugOpBounds(left, top, right, bottom);
	OpPoint center(left + (right - left) * (gridIntervals - x) / gridIntervals,
			top + (bottom - top) * (gridIntervals - y) / gridIntervals);
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
	// !!! assert bounds is square?
	OpDebugImage::drawDoubleFocus();
}

void precision(int p) {
	valuePrecision = p;
	OpDebugImage::drawDoubleFocus();
}

void OpDebugImage::center(int id, bool add) {
	ConstOpPointBoundsPtr pointBoundsPtr = nullptr;
	ConstOpPointPtr pointPtr = nullptr;
	find(id, &pointBoundsPtr, &pointPtr);
	if (pointBoundsPtr)
		return OpDebugImage::drawDoubleCenter(pointBoundsPtr->center(), add);
	if (pointPtr)
		return OpDebugImage::drawDoubleCenter(*pointPtr, add);
}

typedef const OpPointBounds* ConstOpPointBoundsPtr;

void OpDebugImage::find(int id, ConstOpPointBoundsPtr* boundsPtr, ConstOpPointPtr* pointPtr) {
	const OpEdge* edge = nullptr;
	for (const auto& v : { std::cref(segmentEdges), std::cref(temporaryEdges) }) {
		for (const auto& e : v.get()) {
			if (id == e->id)
				edge = e;
		}
	}
	if (!edge) {
		edge = findEdge(id);
		if (edge) {
			if (edge->segment->debugContains(edge))
				segmentEdges.push_back(edge);
			else
				temporaryEdges.push_back(edge);
		}
	}
	if (edge) {
		DRAW_IDS_ON(Edges);
		*boundsPtr = &edge->ptBounds;
		return;
	}
	const OpSegment* segment = nullptr;
	for (auto s : segments) {
		if (id == s->id)
			segment = s;
	}
	if (!segment) {
		segment = findSegment(id);
		if (segment)
			segments.push_back(segment);
	}
	if (segment) {
		DRAW_IDS_ON(Segments);
		*boundsPtr = &segment->ptBounds;
		return;
	}
	const OpIntersection* sect = nullptr;
	for (auto i : intersections) {
		if (id == i->id)
			sect = i;
	}
	if (!sect) {
		sect = findIntersection(id);
		if (sect)
			intersections.push_back(sect);
	}
	if (sect) {
		DRAW_IDS_ON(Intersections);
		// don't change zoom
		*pointPtr = &sect->ptT.pt;
		return;
	}
	const OpIntersection* coin = nullptr;
	for (auto c : coincidences) {
		if (id == c->id)
			coin = c;
	}
	if (!coin) {
		coin = findCoincidence(id);
		if (coin)
			coincidences.push_back(coin);
	}
	if (coin) {
		DRAW_IDS_ON(Coincidences);
		// !!! wrong: add rect formed by both intersections with this id
		*boundsPtr = &coin->segment->ptBounds;
		return;
	}
	OpDebugOut("id " + STR(id) + " not found\n");
}

void OpDebugImage::focus(int id, bool add) {
	ConstOpPointBoundsPtr pointBoundsPtr = nullptr;
	ConstOpPointPtr pointPtr = nullptr;
	find(id, &pointBoundsPtr, &pointPtr);
	if (pointBoundsPtr)
		return OpDebugImage::drawDoubleFocus(*pointBoundsPtr, add);
	if (pointPtr)
		return OpDebugImage::drawDoubleCenter(*pointPtr, add);
}

void addFocus(int id){
	OpDebugImage::focus(id, true);
}

void center(int id) {
	OpDebugImage::center(id, false);
}

void center(float x, float y) {
	OpPoint c(x, y);
	OpDebugImage::drawDoubleCenter(c, false);
}

void focus(int id) {
	OpDebugImage::focus(id, false);
}

void OpDebugImage::focusEdges() {
	if (segmentEdges.empty() && temporaryEdges.empty())
		return;
	OpPointBounds focusRect;
	for (const auto& v : { std::cref(segmentEdges), std::cref(temporaryEdges) }) {
		for (const auto& edge : v.get())
			focusRect.add(edge->ptBounds);
	}
	if (!segmentEdges.empty())
		DRAW_IDS_ON(SegmentEdges);
	if (!temporaryEdges.empty())
		DRAW_IDS_ON(TemporaryEdges);
	DRAW_IDS_ON(Edges);
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

void focusSegments() {
	if (segments.empty())
		return;
	OpPointBounds focusRect = segments.front()->ptBounds;
	for (auto seg : segments)
		focusRect.add(seg->ptBounds);
	DRAW_IDS_ON(Segments);
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

// immediate window commands

// note that operands are not cleared
void clear() {
	OpDebugImage::clearScreen();
	coincidences.clear();
	segmentEdges.clear();
	temporaryEdges.clear();
	intersections.clear();
	OpDebugImage::clearLines();
	if (paths.size() > 2)
		paths.erase(paths.begin() + 2, paths.end());
	segments.clear();
	drawOperandsOn = false;
	drawPathsOn = false;
	DebugOpResetFocus();
}

void clearLines() {
	OpDebugImage::clearLines();
	DebugOpResetFocus();
}

void OpDebugImage::drawValue(const OpPoint& pt, std::string ptStr, uint32_t color) {
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
		for (float scale : { 4, 16 } ) {
			for (int toTheLeft : { -1, 0, 1 } ) {
				for (int toTheTop : { -1, 0, 1 } ) {
					OpVector offset = { 0, 0};
					if (toTheLeft)
						offset.dx = toTheLeft < 0 ? -textBounds.width() - scale : scale;
					if (toTheTop)
						offset.dy = toTheTop < 0 ? -scale : scale;
					SkRect test = trimmed;
					test.offset(offset.dx, offset.dy);
					SkRect skBounds = { 0, 0, bitmapWH, bitmapWH };
					if (allowIntersect ? !skBounds.intersect(test) : !skBounds.contains(test))
						continue;
					int left = std::max(0, (int) test.fLeft);
					int top = std::max(0, (int) test.fTop);
					int right = std::min(bitmapWH, (int) test.fRight);
					int bottom = std::min(bitmapWH, (int) test.fBottom);
					for (int y = top; y < bottom; ++y) {
						for (int x = left; x < right; ++x) {
							if (SK_ColorWHITE != bitmap.getColor(x, y))
								goto loopEnd;
						}
					}
					offscreen.drawString(SkString(ptStr), pt.x + offset.dx,
							pt.y + offset.dy, labelFont, paint);
					return;
				loopEnd:
					;
				}
			}
		}
	}
	offscreen.drawString(SkString("."), pt.x, pt.y, labelFont, paint);
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
	if (drawPathsOn) {
		drawPathPt(operands[0].skPath);
		drawPathPt(operands[1].skPath);
	}
	if (drawSegmentsOn) {
		for (const auto& c : debugGlobalContours->contours) {
			for (const auto& seg : c.segments) {
				if (pointType == seg.c.type)
					continue;
				DebugOpBuild(seg.c.pts[0]);
				DebugOpBuild(seg.c.lastPt());
				// !!! probably need switch to say 'draw control points'
				if (drawControlsOn) {
					for (int index = 1; index < seg.c.pointCount() - 1; ++index)
						DebugOpBuild(seg.c.pts[index]);
				}
			}
		}
	}
	if (drawEdgesOn) {
		if (OpEdgeIntersect::debugActive && drawTemporaryEdgesOn) {
			for (const auto& edge : OpEdgeIntersect::debugActive->edgeParts) {
				DebugOpBuild(edge.start.pt, false);
				DebugOpBuild(edge.end.pt, false);
				if (drawControlsOn && edge.curveSet) {
					for (int index = 1; index < edge.curve_impl.pointCount() - 1; ++index)
						DebugOpBuild(edge.curve_impl.pts[index], false);
				}
			}
			for (const auto& edge : OpEdgeIntersect::debugActive->oppParts) {
				DebugOpBuild(edge.start.pt, true);
				DebugOpBuild(edge.end.pt, true);
				if (drawControlsOn && edge.curveSet) {
					for (int index = 1; index < edge.curve_impl.pointCount() - 1; ++index)
						DebugOpBuild(edge.curve_impl.pts[index], true);
				}
			}
	    }
		auto drawEdges = [](const std::vector<const OpEdge*>& edges) { // lambda
			for (const auto& edge : edges) {
				DebugOpBuild(edge->start.pt);
				DebugOpBuild(edge->end.pt);
				if (drawControlsOn && edge->curveSet) {
					for (int index = 1; index < edge->curve_impl.pointCount() - 1; ++index)
						DebugOpBuild(edge->curve_impl.pts[index]);
				}
			}
		};
		if (drawSegmentEdgesOn)
			drawEdges(segmentEdges);
		if (drawTemporaryEdgesOn)
			drawEdges(temporaryEdges);
	}
	if (drawIntersectionsOn) {
		for (const auto& sect : intersections) {
			DebugOpBuild(sect->ptT.pt);
		}
	}
	if (OpEdgeIntersect::debugActive) {
		const OpSegment* seg = OpEdgeIntersect::debugActive->oppParts[0].segment;
		for (float t : OpEdgeIntersect::debugActive->oppTs)
			DebugOpBuild(seg->c.ptAtT(t), t, true);
		seg = OpEdgeIntersect::debugActive->edgeParts[0].segment;
		for (float t : OpEdgeIntersect::debugActive->edgeTs)
			DebugOpBuild(seg->c.ptAtT(t), t, false);
	}
	if (drawLinesOn) {
		// Check if line intersects visible path, segment, or edge. If so, record point and t
		auto checkEdges = [](const std::vector<const OpEdge*>& edges, const OpRay& ray) { // lambda
			for (const auto& edge : edges)
				DebugOpBuild(*edge, ray.axis, ray.value);
		};
		for (const auto& line : lines) {
			if (drawPathsOn) {
				DebugOpBuild(*operands[0].skPath, line.axis, line.value);
				DebugOpBuild(*operands[1].skPath, line.axis, line.value);
			}
			if (drawSegmentsOn) {
				for (const auto& c : debugGlobalContours->contours) {
					for (const auto& seg : c.segments) {
						if (pointType == seg.c.type)
							continue;
						DebugOpBuild(seg, line.axis, line.value);
					}
				}
			}
			if (OpEdgeIntersect::debugActive && drawTemporaryEdgesOn) {
				for (const auto& edge : OpEdgeIntersect::debugActive->edgeParts)
					DebugOpBuild(edge, line.axis, line.value);
				for (const auto& edge : OpEdgeIntersect::debugActive->oppParts)
					DebugOpBuild(edge, line.axis, line.value);
			}
			if (drawSegmentEdgesOn)
				checkEdges(segmentEdges, line);
			if (drawTemporaryEdgesOn)
				checkEdges(temporaryEdges, line);
		}
	}
	if (drawValuesOn) {
		if (drawTsOn)
			DebugOpDrawT(drawHexOn, valuePrecision);
		else
			DebugOpDrawValue(drawHexOn, valuePrecision);
	}
	DebugOpDrawDiamond();
}

void OpDebugImage::addDiamondToPath(const OpPoint& pt, SkPath& path) {
	SkPath diamond;
	diamond.moveTo(4, 0);
	diamond.lineTo(0, 4);
	diamond.lineTo(-4, 0);
	diamond.lineTo(0, -4);
	diamond.close();
	diamond.offset(pt.x, pt.y);
	path.addPath(diamond);
}

void OpDebugImage::addArrowHeadToPath(const OpLine& line, SkPath& path) {
	if (!drawArrowsOn)
		return;
	const SkPoint arrow[2] = { { -2, 5 }, { 0, 3 } };
	SkPoint scaled[2];
	SkMatrix matrix;
	float lineLen = (line.pts[1] - line.pts[0]).length();
	SkPoint src[2] = { { 0, 0 }, { 0, lineLen } };
	SkPoint dst[2] = { { line.pts[1].x, line.pts[1].y }, { line.pts[0].x, line.pts[0].y } };
	matrix.setPolyToPoly(src, dst, 2);
	matrix.mapPoints(scaled, arrow, 2);
	if (scaled[0].fX != scaled[0].fX)
		OpDebugOut("addArrowHead nan\n");
	path.lineTo(scaled[0]);
	path.lineTo(scaled[1]);
}

void OpDebugImage::add(const OpEdge* edge) {
	for (const auto& v : { std::cref(segmentEdges), std::cref(temporaryEdges) }) {
		for (const auto& e : v.get()) {
			if (e == edge)
				return;
		}
	}
	if (edge->segment->debugContains(edge))
		segmentEdges.push_back(edge);
	else
		temporaryEdges.push_back(edge);
}

void OpDebugImage::add(const OpIntersection* sect) {
	if (intersections.end() != std::find(intersections.begin(), intersections.end(), sect))
		return;
	intersections.push_back(sect);
}

void OpDebugImage::add(Axis axis, float value) {
	lines.emplace_back(axis, value);
}

void OpDebugImage::add(const OpSegment* segment) {
	if (segments.end() != std::find(segments.begin(), segments.end(), segment))
		return;
	segments.push_back(segment);
}

void OpDebugImage::clearSegmentEdges() {
	segmentEdges.clear();
	drawSegmentEdgesOn = false;
}

void OpDebugImage::clearTemporaryEdges() {
	temporaryEdges.clear();
	drawTemporaryEdgesOn = false;
}

void OpDebugImage::clearIntersections() {
	intersections.clear();
	drawIntersectionsOn = false;
}

void OpDebugImage::clearLines() {
	lines.clear();
	drawLinesOn = false;
}

#define HIDE_SHOW_DEFINITION(Thing) \
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

HIDE_SHOW_DEFINITION(Arrows);
HIDE_SHOW_DEFINITION(Bounds)
HIDE_SHOW_DEFINITION(Centers)
HIDE_SHOW_DEFINITION(Chains);
HIDE_SHOW_DEFINITION(Coincidences);
HIDE_SHOW_DEFINITION(Controls);
HIDE_SHOW_DEFINITION(Edges);
HIDE_SHOW_DEFINITION(Grid);
HIDE_SHOW_DEFINITION(IDs);
HIDE_SHOW_DEFINITION(Intersections);
HIDE_SHOW_DEFINITION(Lines);
HIDE_SHOW_DEFINITION(Normals);
HIDE_SHOW_DEFINITION(Paths)
HIDE_SHOW_DEFINITION(Points);
HIDE_SHOW_DEFINITION(SegmentEdges);
HIDE_SHOW_DEFINITION(Segments);
HIDE_SHOW_DEFINITION(Sums);
HIDE_SHOW_DEFINITION(Tangents);
HIDE_SHOW_DEFINITION(TemporaryEdges);
HIDE_SHOW_DEFINITION(Values);
HIDE_SHOW_DEFINITION(Windings);

void hideHex() {
	drawValuesOn = drawHexOn = false;
	OpDebugImage::drawDoubleFocus();
}

void showHex() {
	drawValuesOn = drawHexOn = true;
	OpDebugImage::drawDoubleFocus();
}

void toggleHex() {
	drawHexOn ^= true;
	drawValuesOn = drawHexOn;
	OpDebugImage::drawDoubleFocus();
}

void hideTs() {
	drawTsOn = false;
	OpDebugImage::drawDoubleFocus();
}

void showTs() {
	if (!DebugOpHasT()) {
		OpDebugOut("there are no edge intersect t values\n");
		return;
	}
	drawTsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void toggleTs() {
	if (!drawTsOn && !DebugOpHasT()) {
		OpDebugOut("there are no edge intersect t values\n");
		return;
	}
	drawTsOn ^= true;
	OpDebugImage::drawDoubleFocus();
}

void hideLeft() {
	paths[0] = nullptr;
	drawLeftOn = false;
	OpDebugImage::drawDoubleFocus();
}

void hideRight() {
	paths[1] = nullptr;
	drawRightOn = false;
	OpDebugImage::drawDoubleFocus();
}

void hideOperands() {
	hideLeft();
	hideRight();
}

void showLeft() {
	paths[0] = operands[0].skPath;
	drawLeftOn = true;
	drawPathsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void showOperands() {
	showLeft();
	showRight();
}

void showRight() {
	paths[1] = operands[1].skPath;
	drawRightOn = false;
	drawPathsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void toggleLeft() {
	drawLeftOn ? hideLeft() : showLeft();
}

void toggleRight() {
	drawRightOn ? hideRight() : showRight();
}

void toggleOperands() {
	toggleLeft();
	toggleRight();
}

void OpDebugImage::clearSegments() {
	segments.clear();
	drawSegmentsOn = false;
}

void OpContour::draw() const {
	if (!drawSegmentsOn && !drawEdgesOn)
		drawEdgesOn = true;
	drawIDsOn = true;
	for (auto& segment : segments) {
		if (drawSegmentsOn)
			OpDebugImage::add(&segment);
		for (auto& edge : segment.edges) {
			if (drawEdgesOn)
				OpDebugImage::add(&edge);
		}
	}
	OpDebugImage::drawDoubleFocus();
}


void OpContours::draw() const {
	if (!drawSegmentsOn && !drawEdgesOn)
		drawEdgesOn = true;
	drawIDsOn = true;
	for (auto& contour : contours) {
		for (auto& segment : contour.segments) {
			if (drawSegmentsOn)
				OpDebugImage::add(&segment);
			if (drawEdgesOn && drawSegmentEdgesOn)
				for (auto& edge : segment.edges)
					OpDebugImage::add(&edge);
			if (drawIntersectionsOn)
				for (auto& sect : segment.intersections)
					OpDebugImage::add(&sect);
		}
	}
	OpDebugImage::drawDoubleFocus();
}

void addEdges() {
	drawEdgesOn = true;
	drawSegmentEdgesOn = true;
	debugGlobalContours->draw();
}

void addIntersections() {
	drawIntersectionsOn = true;
	debugGlobalContours->draw();
}

void addSegments() {
	drawSegmentsOn = true;
	debugGlobalContours->draw();
}

OpEdge::~OpEdge() {
	if (!OpDebugPathOpsEnable::inPathOps)
		return;
	assert(!segment->debugContains(this));
	for (auto edge = temporaryEdges.begin(); edge != temporaryEdges.end(); ++edge) {
		if (id == (*edge)->id) {
			temporaryEdges.erase(edge);
			return;
		}
	}
}

void OpEdge::addLink() const {
	drawChain(EdgeLoop::link);
}

void OpEdge::addSum() const {
	drawChain(EdgeLoop::sum);
}

void OpEdge::draw() const {
	OpDebugImage::add(this);
	DRAW_IDS_ON(Edges);
	OpDebugImage::drawDoubleFocus();
}

void OpEdge::drawChain(EdgeLoop edgeLoop) const {
	const OpEdge* chain = this;
	do {
		OpDebugImage::add(chain);
	} while ((chain = EdgeLoop::link == edgeLoop ? chain->nextEdge : chain->nextSum) && this != chain);
	chain = this;
	while ((chain = EdgeLoop::link == edgeLoop ? chain->priorEdge : chain->priorSum) && this != chain) {
		OpDebugImage::add(chain);
	}
	DRAW_IDS_ON(Edges);
	OpDebugImage::drawDoubleFocus();
}

void OpEdge::drawLink() const {
	OpDebugImage::clearTemporaryEdges();
	drawChain(EdgeLoop::link);
}

void OpEdge::drawSum() const {
	OpDebugImage::clearTemporaryEdges();
	drawChain(EdgeLoop::sum);
}

void OpEdges::draw() const {
	OpDebugImage::clearTemporaryEdges();
	for (auto edge : inX)
		OpDebugImage::add(edge);
	OpDebugImage::focusEdges();
}

void OpEdgeIntersect::draw() const {
	if (!edgeParts.size())
		return OpDebugOut("OpEdgeIntersect missing edgeParts\n");
	OpPointBounds focusRect = edgeParts.front().ptBounds;
	for (auto& edge : edgeParts)
		focusRect.add(edge.ptBounds);
	for (auto& edge : oppParts)
		focusRect.add(edge.ptBounds);
	if (oppTs.size() || edgeTs.size()) {
		if (oppTs.size() && !oppParts.size())
			return OpDebugOut("oppTs = " + STR(oppTs.size()) + " but oppParts == 0\n");
		if (edgeTs.size() && !edgeParts.size())
			return OpDebugOut("edgeTs = " + STR(edgeTs.size()) + " but edgeParts == 0\n");
		DRAW_IDS_ON(Points);
		DRAW_IDS_ON(Ts);
	}
	DRAW_IDS_ON(Edges);
	OpDebugImage::drawDoubleFocus(focusRect, false);
}

static void drawEdgeNormal(const OpEdge* edge) {
	OpPoint drawCenter;
	DebugOpPtToPt(edge->center.pt, drawCenter);
	OpEdge copy(*edge);
	const OpCurve& edgeCurve = copy.setCurve();
	OpVector norm = edgeCurve.normal(edge->center.t).normalize() * .5;
	OpLine normal(drawCenter, drawCenter + norm);
	SkPath normalPath;
	normalPath.moveTo(normal.pts[0].x, normal.pts[0].y);
	if (normal.pts[1].x != normal.pts[1].x)
		OpDebugOut("normal nan (edge id)" + STR(edge->id) + "\n");
	normalPath.lineTo(normal.pts[1].x, normal.pts[1].y);
	OpDebugImage::addArrowHeadToPath(normal, normalPath);
	OpDebugImage::drawPath(normalPath);
}

void OpDebugImage::drawEdgeNormals(const std::vector<const OpEdge*>& edges) {
	for (auto edge : edges) {
		drawEdgeNormal(edge);
	}
}

void OpDebugImage::drawEdgeNormals(const std::vector<OpEdge>& edges) {
	for (auto edge : edges) {
		drawEdgeNormal(&edge);
	}
}

static void drawEdgeWinding(const OpEdge* edge) {
	OpPoint drawCenter;
	DebugOpPtToPt(edge->center.pt, drawCenter);
	OpEdge copy(*edge);
	const OpCurve& edgeCurve = copy.setCurve();
	OpVector norm = edgeCurve.normal(edge->center.t).normalize() * .7f;
	OpPoint sumSide = drawCenter + norm;
	OpPoint oppSide = drawCenter - norm;
	std::string sumLeft = STR(edge->sum.left);
	std::string sumRight = STR(edge->sum.right);
	std::string oppLeft = STR(edge->sum.left - edge->winding.left);
	std::string oppRight = STR(edge->sum.right - edge->winding.right);
	SkCanvas textLayer(bitmap);
	SkPaint paint;
	paint.setAntiAlias(true);
	paint.setColor(SK_ColorBLACK);
	textLayer.drawString(SkString(sumLeft), sumSide.x, sumSide.y, labelFont, paint);
	paint.setColor(SK_ColorRED);
	textLayer.drawString(SkString(sumRight), sumSide.x + 20, sumSide.y, labelFont, paint);
	paint.setColor(SK_ColorBLACK);
	textLayer.drawString(SkString(oppLeft), oppSide.x, oppSide.y, labelFont, paint);
	paint.setColor(SK_ColorRED);
	textLayer.drawString(SkString(oppRight), oppSide.x + 20, oppSide.y, labelFont, paint);
}

void OpDebugImage::drawEdgeWindings(const std::vector<const OpEdge*>& edges) {
	for (auto edge : edges) {
		drawEdgeWinding(edge);
	}
}

void OpDebugImage::drawEdgeWindings(const std::vector<OpEdge>& edges) {
	for (auto edge : edges) {
		drawEdgeWinding(&edge);
	}
}

void OpDebugImage::drawLines() {
	SkCanvas offscreen(bitmap);
	SkPaint stroke;
	stroke.setAntiAlias(true);
	stroke.setStyle(SkPaint::kStroke_Style);
	stroke.setColor(SK_ColorRED);
	stroke.setAlpha(127);
	// offscreen.drawPath(linesPath, stroke);
	SkPaint paint;
	paint.setAntiAlias(true);
	for (OpRay& line : lines) {
		// !!! incomplete : draw a line
		if (!drawValuesOn && !drawHexOn)
			continue;
		SkString label = SkString(STR(line.value));
		OpPoint mappedPt;
		if (Axis::vertical == line.axis) {
			DebugOpPtToPt(OpPoint(line.value, 0), mappedPt);
			offscreen.save();
			offscreen.rotate(-90, mappedPt.x, bitmapWH / 2);
			offscreen.drawString(label, mappedPt.x - 10, bitmapWH / 2, labelFont, paint);
			offscreen.restore();
		} else {
			DebugOpPtToPt(OpPoint(0, line.value), mappedPt);
			offscreen.drawString(label, bitmapWH / 2, mappedPt.y - 10, labelFont, paint);
		}
	}
}

void draw(const std::vector<OpEdge*>& _edges) {
	OpDebugImage::clearTemporaryEdges();
	for (auto edge : _edges) {
		OpDebugImage::add(edge);
	}
	OpDebugImage::focusEdges();
}

void draw(const std::vector<OpEdge>& _edges) {
	OpDebugImage::clearTemporaryEdges();
	for (auto& edge : _edges) {
		OpDebugImage::add(&edge);
	}
	OpDebugImage::focusEdges();
}

void draw(Axis axis, float value) {
	OpDebugImage::add(axis, value);
	drawLinesOn = true;
	OpDebugImage::drawDoubleFocus();
}

void draw() {
	OpDebugImage::drawDoubleFocus();
}

void OpSegment::draw() const {
	OpDebugImage::add(this);
	DRAW_IDS_ON(Segments);
	OpDebugImage::drawDoubleFocus();
}

bool OpSegment::debugContains(const OpEdge* edge) const {
	for (auto& e : edges) {
		if (edge == &e)
			return true;
	}
	return false;
}

void OpOutPath::draw() const {
	paths.push_back(skPath);
	drawPathsOn = true;
	OpDebugImage::drawDoubleFocus();
}

void help() {
	OpDebugOut("navigation: l(eft) r(ight) u(p) d(own) i(n) oo(ut)\n");
	OpDebugOut("focus(id) resetFocus() center(id) clear()\n");
	OpDebugOut("showLeft() showRight() showPoints() showIDs()\n");
	OpDebugOut("hideIDs() hideValues() hideHex()\n");
}

void resetFocus() {
	if (!operands.size())
		return OpDebugOut("missing operands\n");
	OpPointBounds focusRect;
	for (unsigned index = 0; index < operands.size(); ++index) {
		SkRect skrect = operands[index].skPath->getBounds();
		focusRect.left = std::min(skrect.fLeft, focusRect.left);
		focusRect.top = std::min(skrect.fTop, focusRect.top);
		focusRect.right = std::max(skrect.fRight, focusRect.right);
		focusRect.bottom = std::max(skrect.fBottom, focusRect.bottom);
	}
	if (focusRect.isFinite())
		OpDebugImage::drawDoubleFocus(focusRect, false);
	else
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
