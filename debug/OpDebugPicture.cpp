// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebug.h"

#if OP_DEBUG_IMAGE

#include "OpDebugPicture.h"
#include "OpContext.h"

bool drawGridLinear = false;

#if 0
void OpDebugAddPoly::add(const LinePts& pts) {    
    std::vector<OpPoint> points { pts.pts[0], pts.pts[1] };
    return picture->add(points);
}

void OpDebugAddPoly::add(const OpRect& r) {
    std::vector<OpPoint> points { 
        { r.left, r.top }, { r.left, r.bottom }, { r.right, r.bottom }, { r.right, r.top } };
    return picture->add(points);
}
#endif

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
        picture->add({ left, y }, { right, y }, this);
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
    auto addSects = [&curve, addPin](OpRoots roots, float xy, Axis axis) {
        for (float root : roots.roots) {
            OpPtT ptAtT = curve.ptTAtT(root);
            ptAtT.pt.choice(axis) = xy;
            addPin(ptAtT);
        }
    };
    addPin(OpPtT(c.data->start, 0));
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.left), picture->focus.left,
            Axis::vertical);
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.top), picture->focus.top,
            Axis::horizontal);
    addSects(curve.axisRayHit(Axis::vertical, picture->focus.right), picture->focus.right,
            Axis::vertical);
    addSects(curve.axisRayHit(Axis::horizontal, picture->focus.bottom), picture->focus.bottom,
            Axis::horizontal);
    addPin(OpPtT(c.data->end, 1));
    // for each span : if middle is inside focus, keep ends of span
    std::sort(sects.begin(), sects.end(), [](const auto& s1, const auto& s2) {
			return s1.sect.t < s2.sect.t; } );
    DebugSect* last = &sects.front();
    for (DebugSect& sect : sects) {
        if (last->sect.t < sect.sect.t) {
            if (last->pin || sect.pin) {
                if (addingFill && last->sect.pt != sect.sect.pt)
                    picture->add(last->sect.pt, sect.sect.pt, this);
            } else {
                OpCurve piece = curve.subDivide(last->sect.t, sect.sect.t);
                piece.setFirstPt(last->sect.pt);
                piece.setLastPt(sect.sect.pt);
                picture->add(piece, this);
                OpDebugPoly& added = picture->polys.back();
                added.tStart = last->sect.t;
                added.tEnd = sect.sect.t;
            }
        }
        last = &sect;
    }
}

void OpDebugAddPoly::add(const OpEdge* e) {
    edge = e;
    segment = nullptr;
    contour = nullptr;
    addingFill = false;
    add(e->curve.c);
}

void OpDebugAddPoly::add(const OpSegment* s) {
    edge = nullptr;
    segment = s;
    contour = nullptr;
    addingFill = false;
    add(s->c.c);
}

// !!! for segments making up area; color comes from contour
void OpDebugAddPoly::add(const OpContour* c) {
    edge = nullptr;
    segment = nullptr;
    contour = c;
    addingFill = true;
    OpPoint last(SetToNaN::dummy);
    for (const PathOpsV0Lib::Curve& curve : c->debugCurves) {
        continueCurve = last == curve.data->start;
        add(curve);
        last = curve.data->end;
    }
    continueCurve = false;
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

void OpDebugPicture::add(const OpCurve& curve, OpDebugAddPoly* polyAdder) {
        // if adding a contour lengthen existing poly it it matches and close the contour as well...
    if (!polyAdder->continueCurve) {
        polys.emplace_back();
        polys.back().c = curve.c;
        if (polyAdder->contour)
            polys.back().color = polyAdder->contour->debugColor;
    }
    OpDebugPoly& poly = polys.back();
    if (polyAdder) {
        poly.edge = polyAdder->edge;
        poly.segment = polyAdder->segment;
        poly.contour = polyAdder->contour;
        poly.isPrimary = true;
        if (poly.contour)
            poly.thickness = OpDebugPoly::fill_thickness;
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
void OpDebugPicture::add(std::vector<OpPoint>& pts ) {
    if (pts.empty())
        return;
    OpPoint last = pts.front();
    for (OpPoint pt : pts) {
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
void OpDebugPicture::add(OpPoint pt1, OpPoint pt2, OpDebugAddPoly* polyAdder) {
    if (pt1 == pt2)
        return;
    std::vector<OpPoint>* lines = nullptr;
    auto getLines = [this, polyAdder, &lines]() {
        polys.emplace_back();
        OpDebugPoly& poly = polys.back();
        poly.edge = polyAdder->edge;
        poly.segment = polyAdder->segment;
        poly.contour = polyAdder->contour;
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
    if (lines->empty() || lines->back() != pt1)
        lines->push_back(pt1);
    lines->push_back(pt2);
}

void OpDebugPicture::addLine(OpPoint pt1, OpPoint pt2) {
    OpDebugPoly& poly = polys.back();
    std::vector<OpPoint>& lines = poly.device;
    lines.push_back(pt1);
    lines.push_back(pt2);
    poly.contours.push_back(2);
}

void OpDebugPicture::addDevice(std::vector<OpPoint>& pts, OpDebugPoly& poly) {
    poly.contours.push_back(pts.size());
    poly.device.insert(poly.device.end(), pts.begin(), pts.end());
}

void OpDebugPicture::addHulls() {
    if (!drawHullsOn)
        return;
    std::vector<OpDebugPoly> toAdd;
    for (OpDebugPoly& poly : polys) {
        if (!poly.edge || !poly.isPrimary)
            continue;
        const OpCurve& c = poly.edge->curve;
        if (c.pointCount() <= 2)
            continue;
        if (!c.ptBounds().width())
            continue;
        if (!c.ptBounds().height())
            continue;
        toAdd.emplace_back();
        toAdd.back().color = OpDebugAlphaColor(40, poly.color);
        toAdd.back().edge = poly.edge;
        std::vector<OpPoint>& hull = toAdd.back().device;
        hull.resize(c.pointCount() + 1);
        for (int index = 0; index < c.pointCount(); ++index) {
            hull[index] = toDevice(c.hullPt(index));
        }
        hull.back() = toDevice(c.hullPt(0));
        toAdd.back().contours.push_back(hull.size());
    }
    polys.insert(polys.begin(), toAdd.begin(), toAdd.end());
}

// span is between this point and last point, if any
void OpDebugPicture::append(OpPoint pt) {
    if (polys.empty())
        polys.emplace_back();
    polys.back().local.push_back(pt);
}

void OpDebugPicture::clear() {
    polys.clear();
    texts.clear();
    points.clear();
    focus = OpRect();
    wh = OpVector();
    threshold = OpVector();
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

// !!! I may have already written this once (or more), but I can't find it :(
bool boundsContains(const OpRect& r, LinePts& line) {
    OpPoint* pts = &line.pts.front();
    OpRect lineBounds(pts[0], pts[1]);
    if (!r.intersects(lineBounds))
        return false;
    if (r.contains(pts[0]) || r.contains(pts[1]))
        return true;
    auto checkRect = [pts, r](float t, XyChoice xy) {
        if (0 > t || t > 1)
            return false;
        float pos = pts[0].choice(xy) * (1 - t) + pts[1].choice(xy) * t;
        return r.ltChoice(xy) < pos && pos < r.rbChoice(xy);
    };
    if (checkRect((r.left - pts[0].x) / (pts[1].x - pts[0].x), XyChoice::inY))
        return true;
    if (checkRect((r.right - pts[0].x) / (pts[1].x - pts[0].x), XyChoice::inY))
        return true;
    if (checkRect((r.top - pts[0].y) / (pts[1].y - pts[0].y), XyChoice::inX))
        return true;
    if (checkRect((r.bottom - pts[0].y) / (pts[1].y - pts[0].y), XyChoice::inX))
        return true;
    return false;
}

void OpDebugPicture::addFittedBottom(std::string s, float xPos, float right, uint32_t color) {
	const int xOffset = 2;
    for (;;) {
        OpDebugText& text = addText(s, OpPoint(), color);
        const NativeTextCache& cache = native_cache(text.cacheIndex);
        if (cache.size.dx + xOffset * 2 <= right - xPos) {
            text.pt.x = xPos + xOffset;
            text.pt.y = screen.bottom - cache.size.dy - xOffset;
            return;
        }
        s.pop_back();
        if (s.empty())
            return;
    }
}

void OpDebugPicture::addFittedSide(std::string s, float yPos, float bottom, uint32_t color) {
	const int xOffset = 2;
    for (;;) {
        OpDebugText& text = addText(s, OpPoint(), color);
        const NativeTextCache& cache = native_cache(text.cacheIndex);
        if (cache.size.dx + xOffset * 2 <= yPos - bottom) {
            text.pt.x = 0;
            text.pt.y = yPos - xOffset;
            text.vertical = true;
            return;
        }
        s.pop_back();
        if (s.empty())
            return;
    }
}

void OpDebugPicture::addGrid() {
    if (!drawGridOn)
        return;
    auto unfixSign = [](int32_t i) {
		return i < 0 ? -i | 0x80000000 : i;
	};
	auto fixSign = [](int32_t i) {
		return i < 0 ? -(i & 0x7fffffff) : i;
	};
	auto hexWorks = [fixSign, unfixSign](std::vector<float>& lines, float fLo, float fHi) {
	    int32_t lo = fixSign(OpDebugFloatToBits(fLo));
	    int32_t hi = fixSign(OpDebugFloatToBits(fHi));
        std::vector<float> linears;
        bool useLinear = drawGridLinear;
        // don't add line for first or last; don't add text for last x, first y
		for (int index = 0; index <= gridIntervals; ++index) {
		    int x = (int32_t) ((int64_t) lo * (gridIntervals - index) / gridIntervals 
                    + (int64_t) hi * index / gridIntervals);
			float fx = OpDebugBitsToFloat(unfixSign(x));
			useLinear |= fabsf(fx) < 1;
            if (lines.empty() || lines.back() != fx)
			    lines.push_back(fx);
            fx = fLo * (gridIntervals - index) / gridIntervals + fHi * index / gridIntervals;
            if (linears.empty() || linears.back() != fx)
			    linears.push_back(fx);
		}
        if (useLinear)
            std::swap(lines, linears);
	};
    std::vector<float> xes;
    hexWorks(xes, focus.left, focus.right);
    std::vector<float> yes;
	hexWorks(yes, focus.top, focus.bottom);
	const uint32_t gridColor = 0x3f000000;
    OpDebugPoly& grid = polys.emplace_back();
    grid.color = gridColor;
    auto xToScreen = [this](float x) {
        return (float) (screen.left + (x - focus.left) * scale);
    };
    for (float fx : xes) {
        if (fx != xes.front() && fx != xes.back())
		    addLine({ xToScreen(fx), screen.top }, { xToScreen(fx), screen.bottom });
    }
    auto yToScreen = [this](float y) {
        return (float) (screen.top + (y - focus.top) * scale);
    };
    for (float fy : yes) {
        if (fy != yes.front() && fy != yes.back())
    		addLine({ screen.left,  yToScreen(fy) }, { screen.right, yToScreen(fy) });
    }
	if (!drawValuesOn)
        return;
    for (size_t index = 0; index + 1 < xes.size(); ++index) {
        float fx = xes[index];
		std::string xValStr = drawHexOn ? OpDebugDumpHex(fx) : STR(fx);
		addFittedBottom(xValStr, xToScreen(fx), xToScreen(xes[index + 1]), gridColor);
    }
    OpDebugText& lastText = texts.back();
    const NativeTextCache& cache = native_cache(lastText.cacheIndex);
	const int xOffset = 2;
    for (size_t index = 1; index < yes.size(); ++index) {
        float fy = yes[index];
		std::string yValStr = drawHexOn ? OpDebugDumpHex(fy) : STR(fy);
        float yScreen = yToScreen(fy);
        if (index == xes.size() - 1)
            yScreen -= cache.size.dy + xOffset;
		addFittedSide(yValStr, yScreen, yToScreen(yes[index - 1]), gridColor);
    }
}

bool OpDebugPicture::touches(const OpRect& bounds) {
    for (OpDebugPoly& poly : polys) {
        if (poly.device.empty())
            continue;
        if (!poly.c.data)
            continue;
        OpRect polyBounds(toDevice(poly.c.data->start), toDevice(poly.c.data->end));
        if (!bounds.intersects(polyBounds))
            continue;
        size_t* contourCounts = &poly.contours.front();
        size_t* contourLast = &poly.contours.back();
        size_t contourCount = *contourCounts;
        OpPoint last = poly.device.front();
        for (size_t index = 0; index < poly.device.size(); ++index) {
            OpPoint devPt = poly.device[index];
            if (last != devPt) {
                LinePts line {{{ last, devPt }}};
                if (boundsContains(bounds, line))
                    return true;
                last = devPt;
            }
            if (--contourCount == 0 && contourCounts != contourLast) {
                contourCount = *++contourCounts;
                last = poly.device[index + 1];
            }
        }
    }
    for (const OpDebugText& text : texts) {
        const NativeTextCache& cache = native_cache(text.cacheIndex);
        OpRect textBounds(text.pt, text.pt + cache.size);
        if (textBounds.intersects(bounds))
            return true;
    }
    return false;
}

OpDebugText& OpDebugPicture::addText(std::string s, OpPoint device, uint32_t color, bool rotated) {
    OpDebugText& text = texts.emplace_back();
    text.cacheIndex = native_addText(s, color);
    text.pt = device;
    text.vertical = rotated;
    return text;
}

void OpDebugPicture::addLabel(std::string s, OpPoint local, uint32_t color) {
    OpVector margin { 4, 4 };
    OpDebugText& text = addText(s, toDevice(local), color, false);
    text.debugLocal = local;
    const NativeTextCache& cache = native_cache(text.cacheIndex);
    // find closest free location
    for (int marginTries = 0; marginTries < 4; ++marginTries) {
        for (int octant = 0; octant < 8; ++octant) {
            OpRect bounds = OpRect(text.pt, text.pt + cache.size);
            switch (octant) {
                case 0:
                    bounds = bounds.offset(margin);
                    break;
                case 1:
                    bounds = bounds.offset(-cache.size - margin);
                    break;
                case 2:
                    bounds = bounds.offset({ margin.dx, -cache.size.dy - margin.dy });
                    break;
                case 3:
                    bounds = bounds.offset({ -cache.size.dx - margin.dx, margin.dy });
                    break;
                case 4:
                    bounds = bounds.offset({ margin.dx, -cache.size.dy / 2 });
                    break;
                case 5:
                    bounds = bounds.offset({ -cache.size.dx / 2, margin.dy });
                    break;
                case 6:
                    bounds = bounds.offset({ -cache.size.dx - margin.dx, -cache.size.dy / 2 });
                    break;
                case 7:
                    bounds = bounds.offset({ -cache.size.dx / 2, -cache.size.dy - margin.dy });
                    break;
                default:
                    OP_ASSERT(0);
            }
            if (!screen.contains(bounds))
                continue;
            if (!touches(bounds)) {
                text.pt = { bounds.left, bounds.top };  // found free-and-clear location
                return;
            }
        }
        margin *= 2;
    }
    text.cacheIndex = native_addText(".", color);
    text.pt += margin;  // if all else fails...
}

void OpDebugPicture::addTangent(OpDebugPoly& poly) {
    OP_ASSERT(poly.contours.size());
    OP_ASSERT(0 < poly.contours[0] && poly.contours[0] <= poly.device.size());
    OP_ASSERT((poly.edge || poly.segment) && poly.isPrimary);
    OpVector span = poly.device[poly.contours[0] - 1] - poly.device.front();
    if (span.length() < 15)
        return;
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
		return;
	}
    OpPoint midTPt = toDevice(curve.ptAtT(.33f));
	LinePts tangent { midTPt, midTPt + tan };
	if (!tangent.pts[1].isFinite()) {
		OpDebugOut(curveStr + " "  + STR(id) + " tangent not finite\n");
		return;
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
	addDevice(tangentPath, poly);
}

void OpDebugPicture::addWinding(OpDebugPoly& poly) {
    if (!poly.edge || !poly.isPrimary)
        return;
    auto add = [poly, this](std::string s, float normSign) {
        size_t cacheIndex = native_addText(s, poly.color);
        const NativeTextCache& cache = native_cache(cacheIndex);
		for (float normLength : { 4.f, 15.f } ) {
			for (float normT : { .58f, .38f, .78f, .18f, .98f } ) {
                OpCurve curve(poly.c, Rotated::no);
				OpVector norm = curve.normal(normT).normalize() * normLength;
				if (!norm.isFinite() || norm == OpVector{ 0, 0 })
					continue;
                OpPoint local = curve.ptAtT(normT);
				OpPoint midTPt = toDevice(local);
                norm *= normSign;
				midTPt += norm;
                OpRect bounds(midTPt, midTPt + cache.size);  // anchor to top left
                if (norm.dx >= 0) {
                    if (norm.dy < 0)  // anchor to bottom left
                        bounds = bounds.offset({0, -cache.size.dy});
                } else if (norm.dy >= 0) // anchor to top right 
                    bounds = bounds.offset({-cache.size.dx, 0});
                else  // anchor to bottom right
                    bounds = bounds.offset(-cache.size); 
                if (!touches(bounds)) {
                    texts.push_back({poly.edge, poly.segment, poly.contour, 
                            {bounds.left, bounds.top}, local, cacheIndex});
                    texts.back().edge = poly.edge;
                    return;
                }
            }
        }
    };
	const OpWinding& sum = poly.edge->sum;
	auto debugImageOut = context->debugContextCallbacks.debugImageWindingOutXFuncPtr;
	add(debugImageOut && sum.isSet() ? (*debugImageOut)(sum.w) : "?", 1);
	std::string sumString = "?";
    const OpWinding& wind = poly.edge->winding;
	if (sum.isSet() || wind.isSet()) {
		if (debugImageOut && !sum.isSet())
			sumString = (*debugImageOut)(wind.w);
        else {
		    OpWinding diffWind(poly.edge->sum.w);
		    context->windingCallbacks.windingSubtractFuncPtr((ContextPtr) context,
                    diffWind.w, wind.w);
		    sumString = debugImageOut ? (*debugImageOut)(diffWind.w) : "";
        }
	}
	add(sumString, -1);
}

#if OP_DEBUG_DUMP
extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

void OpDebugPoly::dump() const {
    std::string s;
    s += "local:" + STR(local.size()) + " ";
    if (edge)
        s += "edge:" + STR(edge->id) + " ";
    if (segment)
        s += "segment:" + STR(segment->id) + " ";
    if (contour)
        s += "contour:" + STR(contour->id) + " ";
    if (1 != thickness)
        s += " thickness:" + STR(thickness) + " ";
    if (debugBlack != color)
        s += "color:" + debugDumpColor(defaultLevel, color) + " ";
    if (0 != tStart)
        s += "tStart:" + STR(tStart) + " ";
    if (1 != tEnd)
        s += "tEnd:" + STR(tEnd) + " ";
    if (isCurveCurve)
        s += "isCurveCurve ";
    if (isPrimary)
        s += "isPrimary ";
    s.pop_back();
    s += "\n";
    for (OpPoint pt : local) {
         s += pt.debugDump(defaultLevel, defaultBase) + "\n";
    }
    OpDebugOut(s);
}

std::string NativeTextCache::debugDump() const {
    std::string s = "\"" + str + "\" ";
    s += "size:" + size.debugDump(defaultLevel, defaultBase) + " ";
    // skip texture
    s += "color:" + debugDumpColor(defaultLevel, color);
    return s;
}

void OpDebugText::dump() const {
    std::string s;
    s += "pt:" + pt.debugDump(defaultLevel, defaultBase) + " ";
    s += "debugLocal:" + debugLocal.debugDump(defaultLevel, defaultBase) + " ";
    s += "cacheIndex:" + STR(cacheIndex) + " ";
    if (vertical) s += "vertical ";
    s += native_debugDump(cacheIndex);
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

void OpDebugPicture::addLabels() {
    for (auto& poly : polys) {
        if (poly.edge && poly.isPrimary && drawEdgesOn) {
            if (drawIDsOn) {
                OpCurve curve(poly.c, Rotated::no);
                OpPoint midTPt = curve.ptAtT(.5);
                addLabel(STR(poly.edge->id), midTPt, poly.color); 
                texts.back().edge = poly.edge;
            }
        }
        if (poly.segment && poly.isPrimary && drawSegmentsOn) {
            if (drawIDsOn) {
                OpCurve curve(poly.c, Rotated::no);
                OpPoint midTPt = curve.ptAtT(.5);
                addLabel(STR(poly.segment->id), midTPt, poly.color); 
                texts.back().segment = poly.segment;
            }
        }
    }
}

void OpDebugPicture::addTangents() {
    if (!drawTangentsOn)
        return;
    for (auto& poly : polys) {
        if ((!poly.edge && !poly.segment) || !poly.isPrimary)
            continue;
        addTangent(poly);
    }
}

void OpDebugPicture::addWindings() {
    if (!drawEdgesOn || !drawWindingsOn)
        return;
    for (auto& poly : polys) {
        if (!poly.edge || !poly.isPrimary)
            continue;
        addWinding(poly);
    }
}

void OpDebugPicture::addPoints() {
    if (!drawPointsOn)
        return;
    auto add = [this](OpDebugPoly* poly, OpPoint local, DebugSprite sprite = DebugSprite::diamond) {
        if (!focus.contains(local))
            return;
        OpPoint device = toDevice(local);
        if (points.end() == std::find_if(points.begin(), points.end(),
                [device](auto& test) { return device == test.device; } )) {
            points.push_back({ poly->edge, poly->segment, poly->contour, local, device, 1, sprite });
            points.back().edge = poly->edge;
            points.back().segment = poly->segment;
            if (drawValuesOn) {
                std::string s = "(";
                s += drawHexOn ? OpDebugDumpHex(local.x) : STR(local.x);
                s += ", ";
                s += drawHexOn ? OpDebugDumpHex(local.y) : STR(local.y);
                s += ")";
                addLabel(s, local, poly->color);
                texts.back().edge = poly->edge;
                texts.back().segment = poly->segment;
            }
        }
    };
    auto addControl = [add](OpDebugPoly* poly, const OpCurve& c) {
        for (int index = 1; index < c.pointCount() - 1; ++index) {
            add(poly, c.hullPt(index));
        }
    };
    points.clear();
    for (auto& poly : polys) {
        if (poly.edge && poly.isPrimary && drawEdgesOn) {
            add(&poly, poly.edge->curve.c.data->start);
            add(&poly, poly.edge->curve.c.data->end);
            if (drawControlsOn)
                addControl(&poly, poly.edge->curve);
            if (drawCentersOn)
                add(&poly, poly.edge->center.pt, DebugSprite::square);

        }
        if (poly.segment && poly.isPrimary && drawSegmentsOn) {
            add(&poly, poly.segment->c.c.data->start);
            add(&poly, poly.segment->c.c.data->end);
            if (drawControlsOn)
                addControl(&poly, poly.segment->c);
        }
    }
    for (OpDebugPoint& dPt : points) {
        auto adder = [this, &dPt](std::vector<OpPoint>& path) {
            for (OpPoint& pt : path) {
                pt += dPt.device;
            }
            if (OpDebugPoly* ePoly = findPoly(dPt.edge))
	            return addDevice(path, *ePoly);
            if (OpDebugPoly* sPoly = findPoly(dPt.segment))
                return addDevice(path, *sPoly);
        };
        switch (dPt.sprite) {
            case DebugSprite::circle: {
                std::vector<OpPoint> path {
                    {  4,  0 }, {  3.696f,  1.531f }, {  2.828f,  2.828f }, {  1.531f,  3.696f }, 
                    {  0,  4 }, { -1.531f,  3.696f }, { -2.828f,  2.828f }, { -3.696f,  1.531f }, 
                    { -4,  0 }, { -3.696f, -1.531f }, { -2.828f, -2.828f }, { -1.531f, -3.696f }, 
                    {  0, -4 }, {  1.531f, -3.696f }, {  2.828f, -2.828f }, {  3.696f, -1.531f }};
                adder(path);
                } break;
            case DebugSprite::diamond: {
	            std::vector<OpPoint> path {{ 4,  0 }, { 0,  4 }, { -4,  0 }, { 0, -4 }, { 4,  0 }};
                adder(path);
                } break;
            case DebugSprite::square: {
	            std::vector<OpPoint> path {{ -4, -4 }, { 4, -4 }, { 4, 4 }, { -4, 4 }, { -4, -4 }};
                adder(path);
                } break;
            case DebugSprite::triangle: {
	            std::vector<OpPoint> path {{ 0, -4 }, { 4, 4 }, { -4, 4 }, { 0, -4 }};
                adder(path);
                } break;
            default:
                OpDebugOut("unknown sprite (%d)" + STR_E(dPt.sprite) + "\n");
        }
    }
}

void OpDebugPicture::colorPolys() {
    for (OpDebugPoly& poly : polys) {
        if (poly.contour) {
        #if 1
            poly.color = OpDebugAlphaColor(31, poly.contour->debugColor);  // !!! convert this to context callout
        #else
            poly.color = poly.contour->debugColor;
        #endif
            continue;
        }
        if (poly.segment) {
            poly.color = poly.segment->debugColor;  // !!! convert this to context callout
            continue;
        }
        if (!poly.edge) {
            poly.color = debugBlack;
            continue;
        }
        const OpEdge& e = *poly.edge;
        PathOpsV0Lib::DebugEdgeType edgeType {
            e.disabled, e.inOutput, Unsortable::none != e.isUnsortable, poly.isCurveCurve, e.ccOverlaps
        };
        PathOpsV0Lib::DebugEdgeColor debugEdgeColor = 
                poly.edge->context()->debugContextCallbacks.debugEdgeColorFuncPtr;
        poly.color = debugEdgeColor ? (*debugEdgeColor)(poly.edge->winding.w, edgeType) : debugBlack;
    }
}

OpDebugPoly* OpDebugPicture::findPoly(const OpEdge* edge) {
    for (OpDebugPoly& poly : polys) {
        if (poly.edge == edge)
            return &poly;
    }
    OP_ASSERT(0);
    return nullptr;
}

OpDebugPoly* OpDebugPicture::findPoly(const OpSegment* segment) {
    for (OpDebugPoly& poly : polys) {
        if (poly.segment == segment)
            return &poly;
    }
    OP_ASSERT(0);
    return nullptr;
}

void OpDebugPicture::bootStrap(OpContext* c) {
    clear();
    addPoly.picture = this;
    context = c;
    double left = 6.2158576999455839;
    double top = 6.2158576999455839;
    double right = 53.784142300054413;
    double bottom = 53.784142300054413;
    left += zoomOffset.dx;
    top += zoomOffset.dy;
    right += zoomOffset.dx;
    bottom += zoomOffset.dy;
// !!! hard-code to above values to bootstrap
//    DebugOpBounds(left, top, right, bottom);
    left *= zoomFactor;
    top *= zoomFactor;
    right *= zoomFactor;
    bottom *= zoomFactor;
    focus = { (float) left, (float) top, (float) right, (float) bottom };
    OpVector localWH { (float) right - (float) focus.left, 
            (float) bottom - (float) focus.top };
    wh = { 1000, 1000 };
    constexpr float subpixels = 4;
    threshold = { localWH.dx / (wh.dx * subpixels), localWH.dy / (wh.dy * subpixels) };
    scale = wh.dx / (right - focus.left);
	for (auto edgeIter = edgeIterator.begin(); edgeIter != edgeIterator.end(); ++edgeIter) {
		const OpEdge* edge = *edgeIter;
		if (!edge->debugDraw)
			continue;
        addPoly.add(edge);
    }
    for (auto segmentIter = segmentIterator.begin(); segmentIter != segmentIterator.end(); ++segmentIter) {
        const OpSegment* segment = *segmentIter;
        addPoly.add(segment);
    }
    for (OpContour* contour : context->contours) {
        PathOpsV0Lib::DebugIsFill debugIsFill = context->debugContextCallbacks.debugIsFillFuncPtr;
        if (debugIsFill && (*debugIsFill)(contour->winding()))
            addPoly.add(contour);
    }
    colorPolys();
    setDevice();
    addHulls();
    addPoints();
    addTangents();
    addLabels();
    addWindings();
    addGrid();
}

void OpDebugPicture::pan(OpVector v) { 
    zoomOffset += v;
    redraw();
}

void OpDebugPicture::redraw() {
    debugGlobalContext = context;
    bootStrap(context);
}

void OpDebugPicture::zoom(float factor) {
    zoomFactor -= factor / 64;
    redraw();
}

#endif
