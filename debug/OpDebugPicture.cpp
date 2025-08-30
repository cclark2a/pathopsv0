// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebug.h"

#if OP_DEBUG_IMAGE

#include "OpDebugPicture.h"
#include "OpContext.h"

void OpDebugAddPoly::add(const LinePts& pts) {    
    std::vector<OpPoint> points { pts.pts[0], pts.pts[1] };
    return picture->add(points);
}

void OpDebugAddPoly::add(const OpRect& r) {
    std::vector<OpPoint> points { 
        { r.left, r.top }, { r.left, r.bottom }, { r.right, r.bottom }, { r.right, r.top } };
    return picture->add(points);
}

struct DebugSect {  // curve intersected with focus rectangle, and intersection pinned to rect
    OpPtT sect;
    bool pin;
};

void OpDebugAddPoly::add(const PathOpsV0Lib::Curve& c) {
    if (c.data->start == c.data->end)
        return;
    // if adding fill, wrap curve to focus bounds
    OpRect bounds(c.data->start, c.data->end);
    auto addVertical = [c, bounds, this](float x) {
        if (!addingFill)
            return;
        float top = std::max(bounds.top, picture->focus.top);
        float bottom = std::min(bounds.bottom, picture->focus.bottom);
        if (top >= bottom)
            return;
        if (c.data->start.y > c.data->end.y)
            std::swap(top, bottom);
        picture->add({ x, top }, { x, bottom }, this);
    };
    auto addHorizontal = [c, bounds, this](float y) {
        if (!addingFill)
            return;
        float left = std::max(bounds.left, picture->focus.left);
        float right = std::min(bounds.right, picture->focus.right);
        if (left >= right)
            return;
        if (c.data->start.x > c.data->end.x)
            std::swap(left, right);
        picture->add({ y, left }, { y, right }, this);
    };
    if (bounds.right <= picture->focus.left)
        return addVertical(picture->focus.left);
    if (bounds.left >= picture->focus.right)
        return addVertical(picture->focus.right);
    if (bounds.bottom <= picture->focus.top)
        return addHorizontal(picture->focus.top);
    if (bounds.top >= picture->focus.bottom)
        return addHorizontal(picture->focus.bottom);
    OpCurve curve(c, Rotated::no);
    if (picture->focus.contains(bounds)) {
        picture->add(curve, this);
        return;
    }
    std::vector<DebugSect> sects;
    // bounds overlaps, but curve may not intersect; find interior ends, intersection with bounds
    auto addPin = [this, &sects](OpPtT end) {
        OpPoint sect = { OpMath::PinSorted(picture->focus.left, end.pt.x, picture->focus.right),
                OpMath::PinSorted(picture->focus.top, end.pt.y, picture->focus.bottom) };
        sects.push_back({{ sect, end.t }, sect != end.pt });
    };
    auto addSects = [this, &sects, &curve, addPin](OpRoots roots) {
        for (float root : roots.roots) {
            addPin(curve.ptTAtT(root));
        }
    };
    addPin(OpPtT(c.data->start, 0));
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.left));
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.top));
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.right));
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.bottom));
    addPin(OpPtT(c.data->end, 1));
    // for each span : if middle is inside focus, keep ends of span
    std::sort(sects.begin(), sects.end(), [](const auto& s1, const auto& s2) {
			return s1.sect.t < s2.sect.t; } );
    DebugSect* last = &sects.front();
    for (DebugSect& sect : sects) {
        if (last->sect.t < sect.sect.t) {
            if (last->pin && sect.pin) {
                if (addingFill) {
                    if (last->sect.pt != sect.sect.pt)
                        picture->add(last->sect.pt, sect.sect.pt, this);
                }
            } else {
                OpCurve piece = curve.subDivide(last->sect.t, sect.sect.t);
                picture->add(piece, this);
                OpDebugPoly& added = picture->polys.back();
                added.tStart = last->sect.t;
                added.tEnd = sect.sect.t;
            }
        }
        last = &sect;
    }
}

void OpDebugAddPoly::add(const OpEdge& e) {
    addingFill = false;
    edge = &e;
    return add(e.curve);
}

void OpDebugAddPoly::add(const OpSegment& s) {
    addingFill = false;
    segment = &s;
    return add(s.c);
}

// !!! for segments making up area; color comes from contour
void OpDebugAddPoly::fill(const OpSegment& s) {
    addingFill = true;
    contour = s.contour;
    add(s.c);
}

struct OpContextSaveThreshold {
    OpContextSaveThreshold(OpContext* c, OpVector threshold) {
        context = c;
        save = context->aliases.threshold;
        context->aliases.threshold = threshold;
        context->aliases.thresholdLength = context->aliases.threshold.length();
    }

    ~OpContextSaveThreshold() {
        context->aliases.threshold = save;
        context->aliases.thresholdLength = context->aliases.threshold.length();
    }

    OpContext* context;
    OpVector save;
};

void OpDebugPicture::add(const OpCurve& curve, OpDebugAddPoly* addPoly) {
    polys.push_back({ curve.c });
    OpDebugPoly& poly = polys.back();
    if (!addPoly) {
        poly.color = debugBlack;
    } else if (const OpEdge* edge = addPoly->edge) {
        poly.edge = edge;
        poly.color = edge->debugColor;
    } else if (const OpSegment* segment = addPoly->segment) {
        poly.segment = segment;
        poly.color = segment->debugColor;
    } else if (const OpContour* contour = addPoly->contour) {
        poly.contour = contour;
        poly.color = contour->debugColor;
    }
    OpContextSaveThreshold save(context, threshold);
    // curve is fully inside focus; split it into lines
    // lengthen curve while longer is linear
    float start = 0;
    float end = 1;
    // split curve until a piece is linear
    append(curve.c.data->start);
    OpCurve piece = curve;
    do {
        while (!piece.isLine()) {
            end = OpMath::Average(start, end);
            piece = curve.subDivide(start, end);
        }
        append(piece.c.data->end);
        float span = end - start;
        OP_ASSERT(span > 0);
        // lengthen the curve while a piece is linear (and while it can be longer)
        bool pieceIsLine = false;
        OpPoint lastEnd(SetToNaN::dummy);
        do {
            start = end;
            end = std::min(1.f, start + span);
            if (start >= end)
                return;
            piece = curve.subDivide(start, end);
            pieceIsLine = piece.isLine();
            if (!pieceIsLine && lastEnd.isFinite()) {
                append(lastEnd);
                start = OpMath::Average(start, end);
                piece = curve.subDivide(start, end);
                break;
            }
            span *= 2;
            lastEnd = piece.c.data->end;
        } while (pieceIsLine && end < 1.f);
    } while (end < 1 || !piece.isLine());
    append(curve.c.data->end);
}

// span is between first and last points, but does not extend last point (unless, see below)
void OpDebugPicture::add(std::vector<OpPoint>& points ) {
    if (points.empty())
        return;
    OpPoint last = points.front();
    for (OpPoint pt : points) {
        if (last == pt)
            continue;
        polys.emplace_back();
        OpDebugPoly& back = polys.back();
        back.cData.start = last;
        back.cData.end = pt;
        back.c = { (ContextPtr) context, &back.cData, sizeof(back.cData), 0 }; 
        OpCurve curve(back.c, Rotated::no);
        add(curve, nullptr);
        last = pt;
    }
}

// for fill only
// span is between points, but does not extend last point unless last point equals first point
void OpDebugPicture::add(OpPoint pt1, OpPoint pt2, OpDebugAddPoly* addPoly) {
    if (pt1 == pt2)
        return;
    std::vector<OpPoint>* lines = nullptr;
    auto getLines = [this, addPoly, &lines]() {
        polys.emplace_back();
        OpDebugPoly& poly = polys.back();
        poly.color = addPoly->color;
        poly.edge = addPoly->edge;
        poly.segment = addPoly->segment;
        poly.contour = addPoly->contour;
        lines = &poly.local;
    };
    if (polys.empty()) {
        getLines();
    } else {
        OpDebugPoly& last = polys.back();
        if (!last.local.empty() && last.local.back() != pt1) {
            getLines();
        } else {
            lines = &last.local;
            OP_ASSERT(lines->size() > 1);
            auto aligned = [](OpPoint p0, OpPoint p1, OpPoint p2) {
                return (p0.x == p1.x && p1.x == p2.x) || (p0.y == p1.y && p1.y == p2.y);
            };
            if (aligned((&lines->back())[-1], pt1, pt2))
                lines->pop_back();
        }
    }
    lines->push_back(pt1);
    lines->push_back(pt2);
}

void OpDebugPicture::addDevice(std::vector<OpPoint>& points, OpDebugPoly& poly) {
    poly.contours.push_back(points.size());
    poly.device.insert(poly.device.end(), points.begin(), points.end());
}

// span is between this point and last point, if any
void OpDebugPicture::append(OpPoint pt) {
    if (polys.empty())
        polys.emplace_back();
    polys.back().local.push_back(pt);
}

void OpDebugPicture::clear() {
    polys.clear();
    focus = OpRect();
    wh = OpVector();
    scale = OpNaN;
}

void OpDebugPicture::setDevice() {
    for (OpDebugPoly& poly : polys) {
        poly.device.reserve(poly.local.size());
        for (OpPoint lPt : poly.local) {
            poly.device.push_back(toDevice(lPt));
        }
        poly.contours.push_back(poly.device.size());
    }
}

OpPoint OpDebugPicture::toLocal(OpPoint pt) {
    return { (float) (pt.x / scale + focus.left), (float) (pt.y / scale + focus.top) };
}

// return local space point in device space
OpPoint OpDebugPicture::toDevice(OpPoint pt) {
    return { (float) ((pt.x - focus.left) * scale), (float) ((pt.y - focus.top) * scale) };
}

#if OP_DEBUG_DUMP
extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

void OpDebugPoly::dump() const {
    std::string s;
    s += "local:" + STR(local.size()) + " thickness:" + STR(thickness) + " color:" + 
            debugDumpColor(defaultLevel, color) + "\n";
    for (OpPoint pt : local) {
         s += pt.debugDump(defaultLevel, defaultBase) + "\n";
    }
    OpDebugOut(s);
}

void OpDebugText::dump() const {
    std::string s;
    s += "pos:" + pos.debugDump(defaultLevel, defaultBase) + " ";
    s += "str:" + str + " ";
    if (vertical) s += "vertical ";
    s.pop_back();
    OpDebugOut(s + "\n");
}

void OpDebugPicture::dump() {
    std::string s;
    s += "focus:" + focus.debugDump(defaultLevel, defaultBase) + " ";
    s += "wh:" + wh.debugDump(defaultLevel, defaultBase) + " ";
    s += "scale:" + STR(scale) + " ";
    s.pop_back();
    OpDebugOut(s + "\n");
    OpDebugOut("polys:\n");
    for (const OpDebugPoly& poly : polys)
        poly.dump();
    OpDebugOut("texts:\n");
    for (const OpDebugText& text : texts)
        text.dump();
}

#endif

#if 1  // for testing
OpDebugPicture debugPicture;
OpDebugAddPoly debugAdd(&debugPicture);

#endif

struct OpDebugBitmap {
    char* bits = nullptr;
    int width = 0;
    int height = 0;
    int rowBytes = 0;
} bitmap;

struct OpDebugFont {
    float getSize() { return size; }
    void setSize(float s) { size = s; }

    std::string name = "Segoe UI";
    float size = 14.f;
} labelFont;

void V0D_AddEdges(OpContext* context) {
    debugPicture.context = context;
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) {
		const OpEdge* edge = *edgeIter;
		if (!edge->debugDraw)
			continue;
        debugAdd.add(edge);
    }
}

void V0D_AddTangents() {
    for (auto& poly : debugPicture.polys) {
        if (!poly.edge && !poly.segment)
            continue;
        OpVector span = poly.device.back() - poly.device.front();
        if (span.length() < 15)
            continue;
        OpCurve curve(poly.c, Rotated::no);
        OpVector tan = curve.tangent(.33f).normalize() * 15;
        if (poly.edge && EdgeMatch::end == poly.edge->which()) {
            tan = -tan;
            poly.color = red;
        }
        std::string curveStr = poly.edge ? "edge" : "segment"; 
        int id = poly.edge ? poly.edge->id : poly.segment->id;
        if (!tan.isFinite() || tan == OpVector{ 0, 0 }) {
		    OpDebugOut(curveStr + " " + STR(id) + " overflow\n");
		    continue;
	    }
        OpPoint midTPt = debugPicture.toDevice(curve.ptAtT(.33f));
	    LinePts tangent { midTPt, midTPt + tan };
	    if (!tangent.pts[1].isFinite()) {
		    OpDebugOut(curveStr + " "  + STR(id) + " tangent not finite\n");
		    continue;
	    }
	    std::vector<OpPoint> tangentPath;
	    tangentPath.push_back(tangent.pts[0]);
	    tangentPath.push_back(tangent.pts[1]);
	    const OpPoint arrow[2] { { -10, -6 }, { 6, 6 } };
        OpVector line = tangent.pts[1] - tangent.pts[0];
	    float r = atan2f(line.dy, line.dx);
        OpPoint stem = tangent.pts[1];
        for (int index = 0; index < 2; ++index) {
            OpPoint rotated = OpPoint(arrow[index].x * cosf(r) - arrow[index].y * sinf(r),  
                    arrow[index].x * sinf(r) + arrow[index].y * cosf(r));
            stem += rotated;
	        tangentPath.push_back(stem);
        }
	    debugPicture.addDevice(tangentPath, poly);
    }
}

void V0D_ClearScreen() {
    debugPicture.clear();
    double left = 6.2158576999455839;
    double top = 6.2158576999455839;
    double right = 53.784142300054413;
    double bottom = 53.784142300054413;
// !!! hard-code to above values to bootstrap
//    DebugOpBounds(left, top, right, bottom);
    debugAdd.picture->focus = { (float) left, (float) top, (float) right, (float) bottom };
    OpVector localWH { (float) right - (float) debugAdd.picture->focus.left, 
            (float) bottom - (float) debugAdd.picture->focus.top };
    debugAdd.picture->wh = { 1000, 1000 };
    constexpr float subpixels = 4;
    debugAdd.picture->threshold = { localWH.dx / (debugAdd.picture->wh.dx * subpixels), 
            localWH.dy / (debugAdd.picture->wh.dy * subpixels) };
    debugAdd.picture->scale = debugAdd.picture->wh.dx / (right - debugAdd.picture->focus.left);
}


#endif
