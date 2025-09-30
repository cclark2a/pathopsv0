// (c) 2025, Cary Clark cclark2@gmail.com
// everything drawn by op debug image

#include "OpDebugPicture.h"
#include "OpCurveCurve.h"
#include "OpSegment.h"

bool drawGridLinear = false;

void PictureWindow::addDevice(std::vector<OpPoint>& pts, DebuggerPoly& poly) {
    poly.contours.push_back(pts.size());
    poly.device.insert(poly.device.end(), pts.begin(), pts.end());
}

void PictureWindow::addHulls() {
    if (!drawHullsOn)
        return;
    std::vector<DebuggerPoly> toAdd;
    for (DebuggerPoly& poly : polys) {
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
void Window::append(OpPoint pt) {
    if (polys.empty())
        polys.emplace_back();
    polys.back().local.push_back(pt);
}

void PictureWindow::clear() {
    clearWindow();
    threshold = OpVector();
    scale = 0;
}

OpPoint PictureWindow::toLocal(OpPoint pt) {
    return { (float) (pt.x / scale + focus.left), (float) (pt.y / scale + focus.top) };
}

// return local space point in device space
OpPoint PictureWindow::toDevice(OpPoint pt) {
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
    if (!drawGridOn)
        return;
    auto unfixSign = [](int32_t i) {
		return i < 0 ? -i | 0x80000000 : i;
	};
	auto fixSign = [](int32_t i) {
		return i < 0 ? -(i & 0x7fffffff) : i;
	};
	auto hexWorks = [fixSign, unfixSign, this](std::vector<float>& lines, float fLo, float fHi) {
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
    DebuggerPoly& grid = polys.emplace_back();
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
    const NativeTextCache& cache = getCache(lastText.cacheIndex);
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

bool PictureWindow::touches(const OpRect& bounds) {
    for (DebuggerPoly& poly : polys) {
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

void PictureWindow::addWinding(DebuggerPoly& poly) {
    if (!poly.edge || !poly.isPrimary)
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
                    texts.push_back({poly.edge, poly.segment, poly.contour, 
                            {bounds.left, bounds.top}, local, cacheIndex});
                    texts.back().edge = poly.edge;
                    return;
                }
            }
        }
    };
	const OpWinding& sum = poly.edge->sum;
	auto debugImageOut = context()->debugContextCallbacks.debugImageWindingOutXFuncPtr;
	add(debugImageOut && sum.isSet() ? (*debugImageOut)(sum.w) : "?", 1);
	std::string sumString = "?";
    const OpWinding& wind = poly.edge->winding;
	if (sum.isSet() || wind.isSet()) {
		if (debugImageOut && !sum.isSet())
			sumString = (*debugImageOut)(wind.w);
        else {
		    OpWinding diffWind(poly.edge->sum.w); // !!! this should be local copy ...
    //        start here;
            // !!! this should be a debug const thingy that can't change user data
		    context()->windingCallbacks.windingSubtractFuncPtr((ContextPtr) context(),
                    diffWind.w, wind.w);
		    sumString = debugImageOut ? (*debugImageOut)(diffWind.w) : "";
        }
	}
	add(sumString, -1);
}

#if OP_DEBUG_DUMP
extern DebugBase defaultBase;
extern DebugLevel defaultLevel;

std::string NativeTextCache::debugDump() const {
    std::string s = "\"" + str + "\" ";
    s += "size:" + size.debugDump(defaultLevel, defaultBase) + " ";
    // skip texture
    s += "color:" + debugDumpColor(defaultLevel, color);
    return s;
}

void OpDebugText::dump(Window& picture) const {
    std::string s;
    s += "pt:" + pt.debugDump(defaultLevel, defaultBase) + " ";
    s += "debugLocal:" + debugLocal.debugDump(defaultLevel, defaultBase) + " ";
    s += "cacheIndex:" + STR(cacheIndex) + " ";
    if (vertical) s += "vertical ";
    s += picture.debugTextDump(cacheIndex);
    OpDebugOut(s + "\n");
}

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

void PictureWindow::addLabels() {
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

void PictureWindow::addTangents() {
    if (!drawTangentsOn)
        return;
    for (auto& poly : polys) {
        if ((!poly.edge && !poly.segment) || !poly.isPrimary)
            continue;
        addTangent(poly);
    }
}

void PictureWindow::addWindings() {
    if (!drawEdgesOn || !drawWindingsOn)
        return;
    for (auto& poly : polys) {
        if (!poly.edge || !poly.isPrimary)
            continue;
        addWinding(poly);
    }
}

void PictureWindow::addPoints() {
    if (!drawPointsOn)
        return;
    auto add = [this](DebuggerPoly* poly, OpPoint local, DebugSprite sprite = DebugSprite::diamond) {
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
    auto addControl = [add](DebuggerPoly* poly, const OpCurve& c) {
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
            if (DebuggerPoly* ePoly = findPoly(dPt.edge))
	            return addDevice(path, *ePoly);
            if (DebuggerPoly* sPoly = findPoly(dPt.segment))
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

void PictureWindow::colorPolys() {
    for (DebuggerPoly& poly : polys) {
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
        OpEdge* ccEdge = context()->ccStorage->debugFind(e.id);
        bool isCurveCurve = ccEdge && e.id == ccEdge->id;
        PathOpsV0Lib::DebugEdgeType edgeType {
            e.disabled, e.inOutput, Unsortable::none != e.isUnsortable, isCurveCurve, e.ccOverlaps
        };
        PathOpsV0Lib::DebugEdgeColor debugEdgeColor = 
                context()->debugContextCallbacks.debugEdgeColorFuncPtr;
        poly.color = debugEdgeColor ? (*debugEdgeColor)(poly.edge->winding.w, edgeType) : debugBlack;
    }
}

bool PictureWindow::drawOne(DebuggerPoly& poly) {
    if (poly.segment && !drawSegmentsOn)
        return false;
    if (poly.edge && !drawEdgesOn)
        return false;
    if (poly.contour && !drawFillOn)
        return false;
    return true;
}

void PictureWindow::redraw() {
    setSize();
    clear();
    OpPointBounds contourBounds;
    for (auto contourIter = contourIterator.begin(); contourIter != contourIterator.end(); ++contourIter) {
        contourBounds.add((*contourIter)->bounds);
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
    float zFactor = tuneThreshold ? 1 : zoomFactor;
    leftTop = center - zFactor * size;
    rightBottom = center + zFactor * size;
    focus = { leftTop, rightBottom };
    OpVector localWH { focus.width(), focus.height() };
    OpVector wh = screen.widthHeight();
    constexpr float subpixels = 4;
    threshold = localWH / (wh * subpixels);
    if (tuneThreshold) {
        threshold *= zoomFactor;
    }
    if (!scale)
        scale = wh.dx / localWH.dx;
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
    for (OpContour* contour : context()->contours) {
        PathOpsV0Lib::DebugIsFill debugIsFill = context()->debugContextCallbacks.debugIsFillFuncPtr;
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

void PictureWindow::move(OpVector v) { 
    zoomOffset -= v / scale;
    draw();
}

void PictureWindow::pan(OpVector v) { 
    zoomOffset += v * 1000 / scale;
    draw();
}

// -1: draw none ; 0: draw all ; > 0 draw matching depth
void PictureWindow::setDepth(int ) {
	OpEdgeStorage* ccStorage = context()->ccStorage;
	int count = ccStorage ? ccStorage->debugCount() : 0;
    int maxDepth = 0;
	for (int index = 0; index < count; ++index) {
		OpEdge* edge = ccStorage->debugIndex(index);
        maxDepth = std::max(maxDepth, edge->debugDepth);
        edge->debugDraw = true;
    }
    depth = std::max(-1, std::min(maxDepth, depth));
    if (depth == 0)  // draw all
        return;
    for (int index = 0; index < count; ++index) {
		OpEdge* edge = ccStorage->debugIndex(index);
		edge->debugDraw = edge->debugDepth < depth && edge->debugCC >= depth;
	}
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

DrawLevel PictureWindow::event(const DebuggerEvent& debuggerEvent) {
    if (debuggerEvent.wheel) {
        int scale = keyModMultiplier(debuggerEvent.keyMods);
        zoom(debuggerEvent.wheel * scale);
        OpDebugOut("zoom:" + STR(zoomFactor)
            + " wheel:" + STR(debuggerEvent.wheel)
            + " scale:" + STR(scale) + "\n");
        return DrawLevel::update;
    }
    if (MouseAction::drag == debuggerEvent.mouseAction) {
        move(debuggerEvent.mouse - debuggerEvent.mouseLast);
        return DrawLevel::update;
    }
    constexpr float pan_factor = 1.f / 8;
    int scale = keyModMultiplier(debuggerEvent.keyMods);
    bool redraw = true;
    switch (uint8_t key = debuggerEvent.key) {
        case (uint8_t) KeyCode::leftArrow:
            pan(OpVector(-pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::upArrow:
            pan(OpVector(0, -pan_factor * scale));
            break;
        case (uint8_t) KeyCode::rightArrow:
            pan(OpVector(pan_factor * scale, 0));
            break;
        case (uint8_t) KeyCode::downArrow:
            pan(OpVector(0, pan_factor * scale));
            break;
        case 'c':
            drawCentersOn ^= true;
            break;
        case 'd':
            if (KeyMods::ctrl == (KeyMods::ctrl & debuggerEvent.keyMods))
                dump();
            else
                setDepth(++depth);
            break;
        case 'D':
            setDepth(--depth);
            break;
        case 'e':
            drawEdgesOn ^= true;
            break;
        case 'f':
            drawFillOn ^= true;
            break;
        case 'g':
            if (drawGridOn && !drawGridLinear)
                drawGridLinear = true;
            else {
                drawGridOn ^= true;
                drawGridLinear = false;
            }
            break;
        case 'h':
            drawHullsOn ^= true;
            break;
        case 'i':
            drawIDsOn ^= true;
            break;
        case 'k':
            drawControlsOn ^= true;
            break;
        case 'p':
            drawPointsOn ^= true;
            break;
        case 's':
            drawSegmentsOn ^= true;
            break;
        case 't':
            drawTangentsOn ^= true;
            break;
        case 'w':
            drawWindingsOn ^= true;
            break;
        case 'v':
            drawValuesOn ^= true;
            break;
        case 'x':
            drawHexOn ^= true;
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            debugPrecision = key - '0';
            break;
        case '-':
            debugPrecision = -1;
            break;
        case '~':
            tuneThreshold ^= true;
            break;
        default:
            redraw = false;
            break;
    }
    return redraw ? DrawLevel::update : DrawLevel::none;
}

void textEvent(Window* window, const DebuggerEvent& debuggerEvent) {
    OP_ASSERT(0);   // !!! start here
}

#if OP_TINY_SKIA

#define OP_X(Thing) \
bool draw##Thing##On = false;
MASTER_LIST
EDGE_BOOL_LIST
ALIAS_LIST
CALLOUT_LIST
#undef OP_X

int gridIntervals = 8;

#endif
