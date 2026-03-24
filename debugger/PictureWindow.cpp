// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "DebuggerState.h"
#include "OpCurveCurve.h"
#include "OpDebugRaster.h"
#include "OpSegment.h"

inline uint32_t OpDebugAlphaColor(uint32_t alpha, uint32_t color) {
	return (alpha << 24) | (color & 0x00FFFFFF);
}

const std::vector<std::string> drawGridStrs {
    "show grid (linear)",
    "show grid (log)",
    "hide grid"
};

std::string GridLabel::labelAt(int index) {
    return drawGridStrs[index];
}

int GridLabel::size() const {
    return (int) drawGridStrs.size();
}

PictureWindow::PictureWindow(DebuggerState* state)
        : DebuggerWindow(state, WheelTarget::zoomAndKeyPan)
        , gridLabel(this) {
    if (SDL_APP_CONTINUE != (state->error = init("picture", { 100, 100 } )))
        OpDebugOut("Couldn't initialize picture window: " + std::string(SDL_GetError()) + "\n");
    else if (SDL_APP_CONTINUE != (state->error = addFont((float) fontSize)))
        OpDebugOut("Couldn't add picture font: " + std::string(SDL_GetError()) + "\n");
}

void PictureWindow::addBounds() {
    if (!drawBounds)
        return;
    std::vector<DebuggerPoly> toAdd;
    auto addRect = [this, &toAdd](DebuggerPoly& poly, OpRect r) {
        toAdd.emplace_back();
        toAdd.back().color = OpDebugAlphaColor(40, poly.color);
        toAdd.back().opType = OpType(poly.opType.edge);
        std::vector<OpPoint>& rectPts = toAdd.back().device;
        rectPts.push_back(toDevice({ r.left, r.top }));
        rectPts.push_back(toDevice({ r.right, r.top }));
        rectPts.push_back(toDevice({ r.right, r.bottom }));
        rectPts.push_back(toDevice({ r.left, r.bottom }));
        rectPts.push_back(toDevice({ r.left, r.top }));
        toAdd.back().contours.push_back(rectPts.size());
    };
    for (DebuggerPoly& poly : polys) {
        if (!poly.isPrimary)
            continue;
        if (debuggerState->showEdges && poly.opType.edge)
            addRect(poly, poly.opType.edge->bounds());
        if (debuggerState->showSegments && poly.opType.segment)
            addRect(poly, poly.opType.segment->c.aliasBounds());
        if (debuggerState->showContours && poly.opType.contour)
            addRect(poly, poly.opType.contour->bounds);
    }
    polys.insert(polys.begin(), toAdd.begin(), toAdd.end());
    OP_DEBUG_CODE(validate());
} 

void PictureWindow::addDevice(std::vector<OpPoint>& pts, DebuggerPoly& poly) {
    poly.contours.push_back(pts.size());
    poly.device.insert(poly.device.end(), pts.begin(), pts.end());
}

void PictureWindow::addEdgeHulls() {
    if (!drawEdgeHulls)
        return;
	for (auto& id : debuggerState->ids) {
        if (!id.drawn)
            continue;
        if (IDType::edge != id.type)
            continue;
        for (const auto& hull : id.edge->hulls.h) {
            if (SectType::endHull == hull.type)
                continue;
            OpPoint device = toDevice(hull.sect.pt);
            if (points.end() != std::find_if(points.begin(), points.end(),
                    [device](auto& test) { return device == test.device; } ))
                continue;
            points.push_back({ id, hull.sect.pt, device, 1, DebugSprite::triangle });
            if (drawValues)
                addPointLabel(hull.sect.pt, id); 
        }
    }
}

void PictureWindow::addHulls() {
    if (!drawHulls)
        return;
    std::vector<DebuggerPoly> toAdd;
    for (DebuggerPoly& poly : polys) {
        if (IDType::edge != poly.opType.type)
            continue;
        if (!poly.opType.edge || !poly.isPrimary)
            continue;
        const OpCurve& c = poly.opType.edge->curve;
        if (c.pointCount() <= 2)
            continue;
        if (!c.aliasBounds().width())
            continue;
        if (!c.aliasBounds().height())
            continue;
        toAdd.emplace_back();
        toAdd.back().color = OpDebugAlphaColor(40, poly.color);
        toAdd.back().opType = OpType(poly.opType.edge);
        std::vector<OpPoint>& hull = toAdd.back().device;
        hull.resize(c.pointCount() + 1);
        for (int index = 0; index < c.pointCount(); ++index) {
            hull[index] = toDevice(c.hullPt(index));
        }
        hull.back() = toDevice(c.hullPt(0));
        toAdd.back().contours.push_back(hull.size());
    }
    polys.insert(polys.begin(), toAdd.begin(), toAdd.end());
    OP_DEBUG_CODE(validate());
}

void PictureWindow::clear() {
    clearWindow();
    scale = 0;
}

OpPoint PictureWindow::toLocal(OpPoint pt) const {
    return { (float) (pt.x / scale + focus.left), (float) (pt.y / scale + focus.top) };
}

// return local space point in device space
OpPoint PictureWindow::toDevice(OpPoint pt) const {
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

void PictureWindow::addFittedBottom(std::string s, float xPos, float right, uint32_t color) {
	const int xOffset = 2;
    for (;;) {
        OpDebugText& text = addText(s, OpPoint(), color);
        const NativeTextCache& cache = getCache(text.cacheIndex);
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

void PictureWindow::addFittedSide(std::string s, float yPos, float bottom, uint32_t color) {
	const int xOffset = 2;
    for (;;) {
        OpDebugText& text = addText(s, OpPoint(), color);
        const NativeTextCache& cache = getCache(text.cacheIndex);
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

void PictureWindow::addGrid() {
    DrawGrid drawGrid = gridLabel.drawGrid();
    if (DrawGrid::none == drawGrid)
        return;
    auto unfixSign = [](int32_t i) {
		return i < 0 ? -i | 0x80000000 : i;
	};
	auto fixSign = [](int32_t i) {
		return i < 0 ? -(i & 0x7fffffff) : i;
	};
	auto hexWorks = [drawGrid, fixSign, unfixSign, this]
            (std::vector<float>& lines, float fLo, float fHi) {
	    int32_t lo = fixSign(OpDebugFloatToBits(fLo));
	    int32_t hi = fixSign(OpDebugFloatToBits(fHi));
        std::vector<float> linears;
        bool useLinear = DrawGrid::linear == drawGrid;
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
    polys.emplace_back();
    DebuggerPoly& grid = polys.back();
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
    OP_DEBUG_CODE(grid.validate());
	if (!drawValues)
        return;
    for (size_t index = 0; index + 1 < xes.size(); ++index) {
        float fx = xes[index];
		std::string xValStr = debuggerState->floatToStr(fx);
		addFittedBottom(xValStr, xToScreen(fx), xToScreen(xes[index + 1]), gridColor);
    }
    OpDebugText& lastText = texts.back();
    const NativeTextCache& cache = getCache(lastText.cacheIndex);
	const int xOffset = 2;
    for (size_t index = 1; index < yes.size(); ++index) {
        float fy = yes[index];
		std::string yValStr = debuggerState->floatToStr(fy);
        float yScreen = yToScreen(fy);
        if (index == xes.size() - 1)
            yScreen -= cache.size.dy + xOffset;
		addFittedSide(yValStr, yScreen, yToScreen(yes[index - 1]), gridColor);
    }
}

bool PictureWindow::touches(const OpRect& bounds) const {
    for (const DebuggerPoly& poly : polys) {
        if (poly.device.empty())
            continue;
        if (!poly.c.data)
            continue;
        OpRect polyBounds(toDevice(poly.c.data->start), toDevice(poly.c.data->end));
        if (!bounds.intersects(polyBounds))
            continue;
        const size_t* contourCounts = &poly.contours.front();
        const size_t* contourLast = &poly.contours.back();
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
        const NativeTextCache& cache = getCache(text.cacheIndex);
        OpRect textBounds(text.pt, text.pt + cache.size);
        if (textBounds.intersects(bounds))
            return true;
    }
    return false;
}

void PictureWindow::addLabel(std::string s, OpPoint local, uint32_t color) {
    OpVector margin { 4, 4 };
    OpDebugText& text = addText(s, toDevice(local), color);
    text.debugLocal = local;
    const NativeTextCache& cache = getCache(text.cacheIndex);
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
    text.cacheIndex = addText(".", color);
    text.pt += margin;  // if all else fails...
}

void PictureWindow::addTangent(DebuggerPoly& poly) {
    OP_ASSERT(poly.contours.size());
    OP_ASSERT(0 < poly.contours[0] && poly.contours[0] <= poly.device.size());
    OP_ASSERT((IDType::edge == poly.opType.type || IDType::contour == poly.opType.type
            || IDType::segment == poly.opType.type) && poly.isPrimary);
    OpVector span = poly.device[poly.contours[0] - 1] - poly.device.front();
    if (span.length() < 15)
        return;
    OpCurve curve(poly.c, Rotated::no);
    OpVector tan = curve.tangent(.33f).normalize() * 15;
    if (IDType::edge == poly.opType.type && EdgeMatch::end == poly.opType.edge->which()) {
        tan = -tan;
        poly.color = red;
    }
    std::string curveStr = IDType::edge == poly.opType.type ? "edge" : 
            IDType::contour == poly.opType.type ? "contour" : "segment"; 
    int id = poly.opType.id;
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

void PictureWindow::addWinding(DebuggerPoly& poly) {
    if (IDType::edge != poly.opType.type || !poly.isPrimary || !poly.c.context)
        return;
    auto add = [poly, this](std::string s, float normSign) {
        size_t cacheIndex = addText(s, poly.color);
        const NativeTextCache& cache = getCache(cacheIndex);
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
                    texts.push_back({ poly.opType, {bounds.left, bounds.top}, local, cacheIndex });
                    return;
                }
            }
        }
    };
	const OpWinding& sum = poly.opType.edge->sum;
	auto debugImageOut = context()->debugContextCallbacks.debugImageWindingOutFuncPtr;
	add(debugImageOut && sum.isSet() ? (*debugImageOut)(sum.w) : "?", 1);
	std::string sumString = "?";
    const OpWinding& wind = poly.opType.edge->winding;
	if (sum.isSet() || wind.isSet()) {
		if (debugImageOut && !sum.isSet())
			sumString = (*debugImageOut)(wind.w);
        else {
		    OpWinding diffWind(poly.opType.edge->sum.w); // local copy ...
		    diffWind.subtract(wind.w);
		    sumString = debugImageOut ? (*debugImageOut)(diffWind.w) : "";
            OpNop();
        }
	}
	add(sumString, -1);
}

#if OP_DEBUG_DUMP
void PictureWindow::dump() {
    std::string s;
    s += "focus:" + focus.debugDump(defaultLevel, defaultBase) + " ";
    s += "scale:" + STR(scale) + " ";
    s.pop_back();
    OpDebugOut(s + "\n");
    OpDebugOut("polys:\n");
    for (const DebuggerPoly& poly : polys)
        poly.dump();
    OpDebugOut("texts:\n");
    for (const OpDebugText& text : texts)
        text.dump(*this);
}
#endif

// does not add id for intersection, since it does not have polygon to draw
void PictureWindow::addIDs() {
    for (auto& poly : polys) {
        if (!poly.isPrimary)
            continue;
        if (!poly.c.context)
            continue;
        if ((IDType::edge != poly.opType.type || !debuggerState->showEdges)
                && (IDType::intersection != poly.opType.type || !debuggerState->showIntersections)
                && (IDType::segment != poly.opType.type || !debuggerState->showSegments)
                && (IDType::contour != poly.opType.type || !debuggerState->showContours))
            continue;
        if (!drawIDs)
            continue;
        OpCurve curve(poly.c, Rotated::no);
        OpPoint midTPt = curve.ptAtT(.5);
        addLabel(STR(poly.opType.id), midTPt, poly.color); 
        texts.back().opType = poly.opType;
    }
}

void PictureWindow::addIntersections() {
    if (!debuggerState->showIntersections)
        return;
	for (auto& id : debuggerState->ids) {
        if (IDType::intersection != id.type)
            continue;
        addLabel(STR(id.intersection->id), id.intersection->ptT.pt, black); 
        texts.back().opType = id;
    }
}

void PictureWindow::addTangents() {
    if (!drawTangents)
        return;
    for (auto& poly : polys) {
        if (!poly.isPrimary)
            continue;
        if ((IDType::edge != poly.opType.type || !debuggerState->showEdges)
                && (IDType::segment != poly.opType.type || !debuggerState->showSegments)
                && (IDType::contour != poly.opType.type || !debuggerState->showContours))
            continue;
        addTangent(poly);
    }
}

void PictureWindow::addTs() {
    if (!debuggerState->showEdges || !drawTs)
        return;
    for (auto& poly : polys) {
        if (!poly.isPrimary)
            continue;
        if (IDType::edge != poly.opType.type)
            continue;
        const OpEdge* edge = poly.opType.edge;
        addLabel(STR(edge->startT), edge->startPt(), poly.color);
        texts.back().opType = poly.opType;
        addLabel(STR(edge->endT), edge->endPt(), poly.color);
        texts.back().opType = poly.opType;
    }
}

void PictureWindow::addWindings() {
    if (!debuggerState->showEdges || !drawWindings)
        return;
    for (auto& poly : polys) {
        if (!poly.isPrimary)
            continue;
        if (IDType::edge != poly.opType.type)
            continue;
        addWinding(poly);
    }
}

void PictureWindow::addOutput() {
    if (!debuggerState->showOutput) 
        return;
    DebugRaster* raster = debuggerState->context->debugRaster;
    if (!raster) 
        return;
    for (const DebugOutput& output : raster->outputs) {
        addPoly.add(output);
    }
}

void PictureWindow::addPointLabel(OpPoint local, OpType& opType) {
    std::string s = "(" + debuggerState->floatToStr(local.x) + ", " 
            + debuggerState->floatToStr(local.y) + ")";
    addLabel(s, local, IDType::edge == opType.type ? edgeColor(*opType.edge) : black);
    texts.back().opType = opType;
}

void PictureWindow::addPoints() {
    if (!drawPoints)
        return;
    auto add = [this](OpType& opType, OpPoint local, DebugSprite sprite = DebugSprite::diamond) {
        if (!local.isFinite())
            return;
        if (!focus.contains(local))
            return;
        OpPoint device = toDevice(local);
        if (points.end() == std::find_if(points.begin(), points.end(),
                [device](auto& test) { return device == test.device; } )) {
            points.push_back({ opType, local, device, 1, sprite });
            if (drawValues)
                addPointLabel(local, opType );
        }
    };
    auto addControl = [add](DebuggerPoly* poly, const OpCurve& c) {
        for (int index = 1; index < c.pointCount() - 1; ++index) {
            add(poly->opType, c.hullPt(index));
        }
    };
    for (auto& poly : polys) {
        if (IDType::edge == poly.opType.type && poly.isPrimary && debuggerState->showEdges) {
            add(poly.opType, poly.opType.edge->curve.c.data->start);
            add(poly.opType, poly.opType.edge->curve.c.data->end);
            if (drawControls)
                addControl(&poly, poly.opType.edge->curve);
            if (drawCenters)
                add(poly.opType, poly.opType.edge->center.pt, DebugSprite::square);

        }
        if (IDType::segment == poly.opType.type && poly.isPrimary && debuggerState->showSegments) {
            add(poly.opType, poly.opType.segment->c.c.data->start);
            add(poly.opType, poly.opType.segment->c.c.data->end);
            if (drawControls)
                addControl(&poly, poly.opType.segment->c);
        }
        if (IDType::intersection == poly.opType.type && poly.isPrimary 
                && debuggerState->showIntersections) {
            add(poly.opType, poly.opType.intersection->ptT.pt);
        }
        // !!! add contours : may require some thought for poly-to-contour-curve mapping
        if (IDType::contour == poly.opType.type && poly.isPrimary && debuggerState->showContours) {
            std::vector<float> extrema;
            OpCurve curve(poly.opType.contour->debugCurve(poly.opType.curveIndex, &extrema), 
                    Rotated::no);
            add(poly.opType, curve.c.data->start);
            add(poly.opType, curve.c.data->end);
            if (drawControls)
                addControl(&poly, curve);
        }
    }
	for (auto& id : debuggerState->ids) {
        if (IDType::intersection != id.type)
            continue;
        add(id, id.intersection->ptT.pt, DebugSprite::square);
    }
}

void PictureWindow::resolvePoints() {
    for (OpDebugPoint& dPt : points) {
        auto adder = [this, &dPt](std::vector<OpPoint>& path) {
            for (OpPoint& pt : path) {
                pt += dPt.device;
            }
            if (DebuggerPoly* ePoly = findPolyByID(dPt.opType.id))
	            return addDevice(path, *ePoly);
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

void PictureWindow::colorPolys() {
    for (DebuggerPoly& poly : polys) {
        if (IDType::contour == poly.opType.type) {
        #if 1
            poly.color = OpDebugAlphaColor(31, poly.opType.contour->debugColor);  // !!! convert this to context callout
        #else
            poly.color = poly.contour->debugColor;
        #endif
            continue;
        }
        if (IDType::segment == poly.opType.type) {
            poly.color = poly.opType.segment->debugColor;  // !!! convert this to context callout
            continue;
        }
        if (IDType::edge == poly.opType.type) {
            poly.color = edgeColor(*poly.opType.edge);
            continue;
        }
        poly.color = black;
    }
}

bool PictureWindow::drawOne(DebuggerPoly& poly) {
    if (IDType::segment == poly.opType.type && !debuggerState->showSegments)
        return false;
    if (IDType::edge == poly.opType.type && !debuggerState->showEdges)
        return false;
    if (IDType::intersection == poly.opType.type && !debuggerState->showIntersections)
        return false;
    if (IDType::contour == poly.opType.type && !drawFill)
        return false;
    return true;
}

uint32_t PictureWindow::edgeColor(const OpEdge& e) {
    bool isCurveCurve = false;
    if (OpEdgeStorage* ccStorage = context()->ccStorage) {
        OpEdge* ccEdge = ccStorage->debugFind(e.id);
        isCurveCurve = ccEdge && e.id == ccEdge->id;
    }
    PathOpsV0Lib::DebugEdgeType edgeType {
        e.disabled, e.inOutput, Unsortable::none != e.isUnsortable, isCurveCurve, e.ccOverlaps
    };
    if (WindingType::uninitialized == e.winding.type)
        return black;
    PathOpsV0Lib::DebugEdgeColor debugEdgeColor = 
            context()->debugContextCallbacks.debugEdgeColorFuncPtr;
    uint32_t color = debugEdgeColor ? (*debugEdgeColor)(e.winding.w, edgeType) : black;
    return color;
}

void PictureWindow::update() {
    setSize();
    clear();
    OpPointBounds contourBounds;
    OpContourIterator contourIter(context());
    for (auto contour : contourIter) {
        if (contour->bounds.isFinite())
            contourBounds.add(contour->bounds);  // (may not be set up early on)
        else {
            for (auto segment : contour->segments) {
                contourBounds.add(segment.c.aliasBounds());
            }
        }
    }
    OpPoint leftTop {0, 0};
    OpPoint rightBottom { 100, 100};
    if (contourBounds.isFinite()) {
        leftTop = { contourBounds.left, contourBounds.top };
        rightBottom = { contourBounds.right, contourBounds.bottom };
        float dWH = contourBounds.width() - contourBounds.height();
        if (dWH > 0) {
            leftTop.y -= dWH / 2;
            rightBottom.y += dWH / 2;
        } else if (dWH < 0) {
            leftTop.x += dWH / 2; 
            rightBottom.x -= dWH / 2; 
        }
    }   
    leftTop += zoomOffset;
    rightBottom += zoomOffset;
// !!! hard-code to above values to bootstrap
//    DebugOpBounds(left, top, right, bottom);
    OpPoint center = (rightBottom + leftTop) / 2;
    OpVector size = (rightBottom - leftTop) / 2;
    leftTop = center - zoomFactor * size;
    rightBottom = center + zoomFactor * size;
    focus = { leftTop, rightBottom };
    OpVector localWH { focus.width(), focus.height() };
    OpVector wh = screen.widthHeight();
    constexpr float subpixels = 4;
    threshold = localWH / (wh * subpixels);
    threshold *= thresholdMultiplier;
    if (!scale)
        scale = wh.dx / localWH.dx;
    PathOpsV0Lib::DebugIsFill debugIsFill = context()->debugContextCallbacks.debugIsFillFuncPtr;
	for (auto& id : debuggerState->ids) {
        if (IDType::edge == id.type && id.drawn)
            addPoly.add(id.edge);
        if (IDType::contour == id.type && debugIsFill && (*debugIsFill)(id.contour->winding())) {
            addPoly.add(id.contour);
        }
        if (IDType::segment == id.type)
            addPoly.add(id.segment);
        if (IDType::intersection == id.type)
            addPoly.add(id.intersection);
    }
    addOutput();
    colorPolys();
    setDevice();
    addBounds();
    addHulls();
    addEdgeHulls();
    addIDs();
    addIntersections();
    addPoints();
    addTangents();
    addTs();
    resolvePoints();
    addWindings();
    addGrid();
}

void PictureWindow::move(OpVector v) { 
    zoomOffset -= v / (float) scale;
    draw();
}

DrawLevel PictureWindow::pan(OpVector v) { 
    zoomOffset += v * 1000 / (float) scale;
    return DrawLevel::update;
}

void PictureWindow::setDevice() {
    for (DebuggerPoly& poly : polys) {
        poly.device.reserve(poly.local.size());
        for (OpPoint lPt : poly.local) {
            poly.device.push_back(toDevice(lPt));
        }
        poly.contours.push_back(poly.device.size());
    }
}

void PictureWindow::zoom(int factor) {
    zoomer -= factor;
    zoomFactor = powf(2, zoomer / 32.f);
    draw();
}

DrawLevel PictureWindow::doWheel(const DebuggerEvent& debuggerEvent, int delta) {
    int scale = DebuggerEvent::KeyModMultiplier(debuggerEvent.keyMods);
    if (WheelTarget::threshold == wheelTarget) {
        thresholdWheel -= debuggerEvent.wheel * scale;
        thresholdMultiplier = powf(2, thresholdWheel / 32.f);
    } else {
        zoom(delta * scale);
        OpDebugOut("zoom:" + STR(zoomFactor)
                + " wheel:" + STR(delta)
                + " scale:" + STR(scale) + "\n");
    }
    return DrawLevel::update;
}

DrawLevel PictureWindow::event(const DebuggerEvent& debuggerEvent) {    
    if (DrawLevel common = debuggerState->eventCommon(debuggerEvent); DrawLevel::none != common)
        return common;
    if (debuggerEvent.wheel)
        return doWheel(debuggerEvent, debuggerEvent.wheel);
    if (MouseAction::drag == debuggerEvent.mouseAction) {
        move(debuggerEvent.mouse - debuggerEvent.mouseLast);
        return DrawLevel::update;
    }
    return DrawLevel::none;
}

void PictureWindow::playback(const char*& str) {
    playbackCommon(str);
    OpDebugRequired(str, "gridLabel");
    gridLabel.lastIndex = (int) OpDebugReadSizeT(str);
    DEBUG_SET_COMMON_STRUCT(zoomOffset);
    scale = OpDebugReadNamedFloat(str, "scale");  // factor to go from local to device (zero is uninitialized)
    DEBUG_SET_FLOAT(scale, thresholdMultiplier);
    DEBUG_SET_FLOAT(thresholdMultiplier, zoomFactor);
    DEBUG_SET_REQUIRED_VALUE(zoomFactor, thresholdWheel);
    DEBUG_SET_REQUIRED_VALUE(thresholdWheel, zoomer);
    DEBUG_SET_REQUIRED_VALUE(zoomer, gridIntervals);
    DEBUG_SET_BOOL(gridIntervals, drawBounds);
    DEBUG_SET_BOOL(drawBounds, drawCenters);
    DEBUG_SET_BOOL(drawCenters, drawControls);
    DEBUG_SET_BOOL(drawControls, drawEdgeHulls);
    DEBUG_SET_BOOL(drawEdgeHulls, drawFill);
    DEBUG_SET_BOOL(drawFill, drawHulls);
    DEBUG_SET_BOOL(drawHulls, drawIDs);
    DEBUG_SET_BOOL(drawIDs, drawPoints);
    DEBUG_SET_BOOL(drawPoints, drawTangents);
    DEBUG_SET_BOOL(drawTangents, drawTs);
    DEBUG_SET_BOOL(drawTs, drawValues);
    DEBUG_SET_BOOL(drawValues, drawWindings);
}

#define DEBUG_DUMP_ENUM_VALUE(lastField, thisValue) \
    ASSERT_ORDERED(lastField, thisValue); \
    s += #thisValue ":" + STR((int) thisValue) + " "

std::string PictureWindow::record() {
    std::string s;
    DebugLevel l = DebugLevel::file;
    DebugBase b = DebugBase::hex;
    s += recordCommon();
    s += "gridLabel:" + STR(gridLabel.lastIndex) + " ";
    DEBUG_DUMP_COMMON_STRUCT(zoomOffset);
    if (!OpMath::IsDebugNaN((float) scale))
        s += debugValue(DebugLevel::error, b, "scale", (float) scale) + " ";
    DEBUG_DUMP_FLOAT(scale, thresholdMultiplier);
    DEBUG_DUMP_FLOAT(thresholdMultiplier, zoomFactor);
    DEBUG_DUMP_REQUIRED_VALUE(zoomFactor, thresholdWheel);
    DEBUG_DUMP_REQUIRED_VALUE(thresholdWheel, zoomer);
    DEBUG_DUMP_REQUIRED_VALUE(zoomer, gridIntervals);
    DEBUG_DUMP_BOOL(gridIntervals, drawBounds);
    DEBUG_DUMP_BOOL(drawBounds, drawCenters);
    DEBUG_DUMP_BOOL(drawCenters, drawControls);
    DEBUG_DUMP_BOOL(drawControls, drawEdgeHulls);
    DEBUG_DUMP_BOOL(drawEdgeHulls, drawFill);
    DEBUG_DUMP_BOOL(drawFill, drawHulls);
    DEBUG_DUMP_BOOL(drawHulls, drawIDs);
    DEBUG_DUMP_BOOL(drawIDs, drawPoints);
    DEBUG_DUMP_BOOL(drawPoints, drawTangents);
    DEBUG_DUMP_BOOL(drawTangents, drawTs);
    DEBUG_DUMP_BOOL(drawTs, drawValues);
    DEBUG_DUMP_BOOL(drawValues, drawWindings);
    return s;
}
