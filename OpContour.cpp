#include "OpContour.h"
#include "OpEdges.h"
#include "OpSegments.h"
#include "PathOps.h"

static const OpOperator OpInverse[+OpOperator::ReverseSubtract + 1][2][2] {
    //                inside minuend                                  outside minuend
    //  inside subtrahend     outside subtrahend         inside subtrahend   outside subtrahend
    { { OpOperator::Subtract, OpOperator::Intersect }, { OpOperator::Union, OpOperator::ReverseSubtract } },
    { { OpOperator::Intersect, OpOperator::Subtract }, { OpOperator::ReverseSubtract, OpOperator::Union } },
    { { OpOperator::Union, OpOperator::ReverseSubtract }, { OpOperator::Subtract, OpOperator::Intersect } },
    { { OpOperator::ExclusiveOr, OpOperator::ExclusiveOr }, { OpOperator::ExclusiveOr, OpOperator::ExclusiveOr } },
    { { OpOperator::ReverseSubtract, OpOperator::Union }, { OpOperator::Intersect, OpOperator::Subtract } },
};

OpContours::OpContours(OpInPath& l, OpInPath& r, OpOperator op) 
    : leftIn(l)
    , rightIn(r)
    , opIn(op)
{
    _operator = OpInverse[+op][l.isInverted()][r.isInverted()];
    sectStorage = new OpSectStorage;
#if OP_DEBUG
    debugInPathOps = false;
    debugInClearEdges = false;
    debugResult = nullptr;
    debugExpect = OpDebugExpect::unknown;
#endif
}

bool OpContour::addClose() {
    if (!segments.size())
        return false;
    OpPoint curveStart = segments.front().c.pts[0];
    OpPoint lastPoint = segments.back().c.lastPt();
    if (lastPoint != curveStart) {
        CurvePts curvePts = {{ lastPoint, curveStart }, 1 };
        segments.emplace_back(curvePts, OpType::line  
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
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[2]);
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }, weight };
        if (whole.isLinear(OpType::conic)) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        } else
            segments.emplace_back(whole, OpType::conic  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        CurvePts partPts = conic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear(OpType::conic)) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        } else 
            segments.emplace_back(partPts, OpType::conic  OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
}

void OpContour::addCubic(const OpPoint pts[4]) {
    // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
    OP_ASSERT(pts[0] != pts[3]); // !!! detect possible degenerate to code from actual test data
    OpTightBounds bounds;
    OpCubic cubic(pts);
    bounds.calcBounds(cubic);
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[3]);

    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2], pts[3] }};
        if (whole.isLinear(OpType::cubic)) {
            LinePts linePts = {{ whole.pts[0], whole.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        } else
            segments.emplace_back(whole, OpType::cubic  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[3], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        CurvePts partPts = cubic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear(OpType::cubic)) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(start.reason, end.reason, this));
        } else
            segments.emplace_back(partPts, OpType::cubic  
                    OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
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
    LinePts linePts = {{ pts[0], pts[1] }};
    segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
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
    std::vector<ExtremaT> extrema = bounds.findExtrema(pts[0], pts[2]);
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }};
        if (whole.isLinear(OpType::quad)) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        } else
            segments.emplace_back(whole, OpType::quad  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::startPt));
        CurvePts partPts = quad.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear(OpType::quad)) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt, this));
        } else 
            segments.emplace_back(partPts, OpType::quad  
                    OP_DEBUG_PARAMS(start.reason, end.reason, this));
        start = end;
    }
}


#if OP_DEBUG
void OpContour::debugComplete() {
    id = contours->id++;
}
#endif

void OpContour::finish() {
// after all segments in contour have been allocated
    OP_ASSERT(segments.size() > 2);
    OpSegment* last = &segments.back();
    last->contour = this;
    last->resortIntersections = true; // 1 gets added before 0
    for (auto& segment : segments) {
        segment.contour = this;
        segment.winding = operand;
        OpPoint firstPoint = segment.c.pts[0];
        OpIntersection* sect = segment.addSegBase({ firstPoint, 0}  
                OP_DEBUG_PARAMS(SECT_MAKER(segStart), segment.debugStart, last));
        OpIntersection* oSect = last->addSegBase({ firstPoint, 1}  
                OP_DEBUG_PARAMS(SECT_MAKER(segEnd),  last->debugEnd, &segment));
        sect->pair(oSect);
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

// build list of linked edges
// if they are closed, done
// if not, match up remainder
// make sure normals point same way
// prefer smaller total bounds
bool OpContours::assemble(OpOutPath path) {
    OpEdges edges(*this, EdgesToSort::byBox);  // collect active edges and sort them
    OpEdges unsectables(*this, EdgesToSort::unsectable);  // collect unsectables as well
    if (!edges.inX.size())
        return true;
    for (auto edge : edges.inX) {
        edge->setActive();
    }
    for (auto unsectable : unsectables.inX) {
        unsectable->setActive();
    }
    for (auto unsortable : unsortables) {
        unsortable->setActive();
    }
#if 01 && OP_DEBUG
    ::clear();
    ::hideSegmentEdges();
    ::hideIntersections();
    edges.debugDraw();
    unsectables.debugAdd();
    ::add(unsortables);
    ::redraw();
    OpDebugOut("");
#endif
    // match up edges that have only a single possible prior or next link, and add them to new list
    std::vector<OpEdge*> linkups;
    for (auto& leftMost : edges.inX) {
        if (!leftMost->winding.visible())
            continue;   // likely marked as part of a loop below
        if (!leftMost->isActive())  // check if already saved in linkups
            continue;
        OP_ASSERT(EdgeLink::unlinked == leftMost->priorLink);
        OP_ASSERT(EdgeLink::unlinked == leftMost->nextLink);
        leftMost->whichEnd = EdgeMatch::start;
        OpEdge* linkup = leftMost->linkUp(EdgeMatch::start, leftMost);
        if (!linkup->containsLink(leftMost)) {
            linkup->output(path);
            OpEdge* pFirst = leftMost->prepareForLinkup();
            linkups.emplace_back(pFirst);
            continue;
        }
        if (linkup != leftMost)
            linkup->lastEdge = leftMost;
        if (leftMost->isClosed(linkup)) {
            leftMost->output(path);  // emit the contour
            continue;
        }
        OpEdge* first = linkup ? linkup : leftMost;
        OpDebugPlayback(first, 450);
        OpEdge* newLast = leftMost->linkUp(EdgeMatch::end, first);
        if (!leftMost->containsLink(newLast)) {
            newLast->output(path);
            OpEdge* pFirst = leftMost->prepareForLinkup();
            linkups.emplace_back(pFirst);
            continue;
        }
        if (newLast != leftMost) {
            if (linkup)
                linkup->lastEdge = newLast;
            // if a closed loop is formed, just output that
            // if it is nearly a loop and can be closed with a unsortable edge, do that
            // !!! TODO : find direction of loop at add 'reverse' param to output if needed
            //     direction should consider whether edge normal points to inside or outside
        }
        if (newLast->isClosed(first) || closeGap(newLast, first, unsectables.inX)) {
            first->output(path);  // emit the contour
            continue;
        }
        first = first->prepareForLinkup();
        linkups.emplace_back(first);
    }
    for (auto linkup : linkups) {
        OP_ASSERT(!linkup->isLoop(WhichLoop::prior, LeadingLoop::will));
        OP_ASSERT(!linkup->isLoop(WhichLoop::next, LeadingLoop::will));
        OP_ASSERT(linkup->lastEdge);
        OP_ASSERT(!linkup->priorEdge);
        do {
            linkup->setActive();
        } while ((linkup = linkup->nextEdge));
    }
#if 01 && OP_DEBUG
    OpDebugOut("");
#endif
    // !!! to do : find edges to fill gaps in remaining pieces, starting with the largest
    for (auto linkup : linkups) {
        if (!linkup->isActive())
            continue;
        if (!linkup->lastEdge->isClosed(linkup) && !closeGap(linkup->lastEdge, linkup,
                unsectables.inX) && !linkup->matchLink(linkups, unsectables.inX))
            return false;   // if edges form loop with tail, fail
        // determine direction of linked edges, if any (a straight line won't have a direction)
        linkup->output(path);  // emit the contour
    }
    return true;
}

bool OpContours::closeGap(OpEdge* last, OpEdge* first, std::vector<OpEdge*>& unsectInX) {
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
    for (auto edge : unsectInX) {
        if (!edge->isActive())
            continue;
        if (connectBetween(edge)) {
            OP_ASSERT(EdgeSum::unsectable == edge->sumType);
            edge->clearActive();   // only use edge once
            for (auto pals : edge->pals) {
                pals->clearActive();
            }
            return true;
        }
    }
    for (auto edge : unsortables) {
        if (connectBetween(edge)) {
            OP_ASSERT(EdgeSum::unsortable == edge->sumType);
            edge->sumType = EdgeSum::unset;   // only use edge once
            return true;
        }
    }
    return false;
}

void OpContours::finishAll() {
    for (auto& contour : contours)
        contour.finish();
}

static const bool OutInverse[+OpOperator::ReverseSubtract + 1][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};

// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(contourList, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.

bool OpContours::pathOps(OpOutPath result) {
    if (!build(leftIn, OpOperand::left))  // builds monotonic segments, and adds 0/1 sects
        OP_DEBUG_FAIL(*this, false);
    if (!build(rightIn, OpOperand::right))
        OP_DEBUG_FAIL(*this, false);
    finishAll();
    setBounds();    // !!! check to see if this is used
    OpSegments sortedSegments(*this);
    if (!sortedSegments.inX.size()) {
        result.setEmpty();
        OP_DEBUG_SUCCESS(*this, true);
    }
    sortedSegments.findCoincidences();  // check for exact curves and full lines
    sortedSegments.findLineCoincidences();  // check for partial h/v lines
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        OP_DEBUG_FAIL(*this, false);
//    resolvePoints();    // multiple points may have same t value
//    calcBounds();   // resolve points may have changed tight bounds
    sortIntersections();
    makeEdges();
//    OpEdges sortedEdges(contourList, EdgesToSort::byBox);
//    if (!sortedEdges.inX.size()) {
//        result.setEmpty();
//        OP_DEBUG_SUCCESS(contourList, true);
//    }
//    if (FoundIntersections::fail == sortedEdges.findIntersections())
//        OP_DEBUG_FAIL(contourList, false);
//    missingCoincidence();  // add intersections for indirect coincidence
    // at this point, edges curves broken at extrema and inflection;
    //   intersections are ptT for each found crossing
 //   sortIntersections();    // !!! should do nothing if intersections are unchanged
//    resolvePoints();    // added coincident points may have multiple pts with single t
//    intersectEdge();  // combine edge list and intersection list
//    if (!resolveCoincidence())  // leave at most one active for each pair of coincident edges
//        OP_DEBUG_FAIL(contourList, false);
    OpEdges windingEdges(*this, EdgesToSort::byCenter);
    FoundWindings foundWindings = windingEdges.setWindings(this);  // walk edge list, compute windings
    if (FoundWindings::fail == foundWindings)
        OP_DEBUG_FAIL(*this, false);
    apply();  // suppress edges which don't meet op criteria
    // A segment may contain multiple intersections with the same t and different points.
    // If found, replace all matching points with the average, in this and the intersected segment.
    // !!! probably not needed if done earlier (if needed, must be rewritten)
//    resolvePoints();
    if (!assemble(result))
        OP_DEBUG_FAIL(*this, false);
    bool inverseFill = OutInverse[+_operator][leftIn.isInverted()][rightIn.isInverted()];
    result.setInverted(inverseFill);
    OP_DEBUG_SUCCESS(*this, true);
}

