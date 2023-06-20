#include "OpContour.h"
#include "OpEdge.h"

bool OpContour::addClose() {
    if (!segments.size())
        return false;
    OpPoint curveStart = segments.front().c.pts[0];
    OpPoint lastPoint = segments.back().c.lastPt();
    if (lastPoint != curveStart) {
        CurvePts curvePts = {{ lastPoint, curveStart }, 1 };
        segments.emplace_back(curvePts, lineType  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
    }
    return true;
}

void OpContour::addConic(const OpPoint pts[3], float weight) {
    if (pts[0] == pts[2])   // !!! should be fill only, not frame
        return;
    OpTightBounds bounds;
    OpConic conic(pts, weight);
    bounds.calcBounds(conic);
    std::vector<ExtremaT> extrema = bounds.findExtrema();
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }, weight };
        segments.emplace_back(whole, conicType  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    ExtremaT end;
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        if (index < extrema.size())
            end = extrema[index];
        else
            end = {{ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::endPt)};
        CurvePts partPts = conic.subDivide(start.ptT, end.ptT);
        segments.emplace_back(partPts, conicType  OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
}

void OpContour::addCubic(const OpPoint pts[4]) {
    // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
    assert(pts[0] != pts[3]); // !!! detect possible degenerate to code from actual test data
    OpTightBounds bounds;
    OpCubic cubic(pts);
    bounds.calcBounds(cubic);
    std::vector<ExtremaT> extrema = bounds.findExtrema();
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2], pts[3] }, 1};
        segments.emplace_back(whole, cubicType  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    ExtremaT end;
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        if (index < extrema.size())
            end = extrema[index];
        else
            end = {{ pts[3], 1 }  OP_DEBUG_PARAMS(SectReason::endPt)};
        CurvePts partPts = cubic.subDivide(start.ptT, end.ptT);
        segments.emplace_back(partPts, cubicType  OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
    dumpDetail();
}

OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpEdge* edge, const OpEdge* oEdge)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, SectFlavor::none, 0  OP_DEBUG_PARAMS(maker, line, file, reason, 
            edge->id, oEdge->id));
    return next;
}

void OpContour::addLine(const OpPoint pts[2]) {
    if (pts[0] == pts[1])   // !!! should be fill only, not frame
        return;
    CurvePts curvePts = {{ pts[0], pts[1] }, 1 };
    segments.emplace_back(curvePts, lineType   
            OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
}

OpContour* OpContour::addMove(const OpPoint pts[1]) {
            // !!! if frame paths are supported, don't add close unless fill is set
    if (addClose())
        contours->contours.emplace_back(contours, operand);
    // keep point as its own segment in the future?
    return &contours->contours.back();
}

OpIntersection* OpContour::addSegSect(const OpPtT& t, OpSegment* seg, SectFlavor flavor, int cID
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason,
        const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, flavor, cID  OP_DEBUG_PARAMS(maker, line, file, reason, seg->id, oSeg->id));
    return next;
}

void OpContour::addQuad(const OpPoint pts[3]) {
    if (pts[0] == pts[2])   // !!! should be fill only, not frame
        return;
    OpTightBounds bounds;
    OpQuad quad(pts);
    bounds.calcBounds(quad);
    std::vector<ExtremaT> extrema = bounds.findExtrema();
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }, 1};
        segments.emplace_back(whole, quadType  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    ExtremaT end;
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        if (index < extrema.size())
            end = extrema[index];
        else
            end = {{ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::startPt)};
        CurvePts partPts = quad.subDivide(start.ptT, end.ptT);
        segments.emplace_back(partPts, quadType  OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
}


#if OP_DEBUG
void OpContour::debugComplete() {
    id = contours->id++;
}
#endif

void OpContour::finish() {
    (void) addClose();
// after all segments in contour have been allocated
    assert(segments.size() > 2);
    OpSegment* last = &segments.back();
    for (auto segment : segments) {
        segment.contour = this;
        segment.winding = operand;
        OpPoint firstPoint = segment.c.pts[0];
        segment.addSegSect({ firstPoint, 0}  OP_DEBUG_PARAMS(SECT_MAKER(segStart), 
                segment.debugStart, last));
        last->addSegSect({ firstPoint, 1}  OP_DEBUG_PARAMS(SECT_MAKER(segEnd), 
                last->debugEnd, &segment));
        last = &segment;
    }
}

OpIntersection* OpContours::allocateIntersection() {
    if (sectStorage->used == ARRAY_COUNT(sectStorage->storage)) {
        OpSectStorage* next = new OpSectStorage;
        next->next = sectStorage;
        sectStorage = next;
    }
    return &sectStorage->storage[sectStorage->used++];
}

bool OpContours::closeGap(OpEdge* last, OpEdge* first) {
    OpPoint start = first->whichPtT(EdgeMatch::start).pt;
    OpPoint end = last->whichPtT(EdgeMatch::end).pt;
    auto connectBetween = [=](OpEdge* edge) {  // lambda
        if (start != edge->start.pt && end != edge->start.pt)
            return false;
        if (start != edge->end.pt && end != edge->end.pt)
            return false;
        edge->linkNextPrior(first, last);
        return true;
    };
    for (auto edge : unsortables) {
        if (connectBetween(edge)) {
            edge->unsortable = false;   // only use edge once
            return true;
        }
    }
    return false;
}

void OpContours::finish(OpContour& contour) {
    if (!contour.segments.size())
        contours.pop_back();
    else 
        contour.finish();
}
