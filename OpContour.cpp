#include "OpContour.h"
#include "OpJoiner.h"
#include "OpSegments.h"
#include "OpWinder.h"
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
    opOperator = OpInverse[+op][l.isInverted()][r.isInverted()];
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
        LinePts linePts = {{ lastPoint, curveStart }};
        segments.emplace_back(linePts, this  
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
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
        OpCurve whole(pts, weight, OpType::conic);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::conic, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        OpCurve partPts = conic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else 
            segments.emplace_back(partPts, OpType::conic, this
                    OP_DEBUG_PARAMS(start.reason, end.reason));
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
        OpCurve whole(pts, OpType::cubic);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::cubic, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[3], 1 }  OP_DEBUG_PARAMS(SectReason::endPt));
        OpCurve partPts = cubic.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[3] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  OP_DEBUG_PARAMS(start.reason, end.reason));
        } else
            segments.emplace_back(partPts, OpType::cubic, this  
                    OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
}

OpIntersection* OpContour::addEdgeSect(const OpPtT& t, OpSegment* seg
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
        const OpEdge* edge, const OpEdge* oEdge)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, 0, 0  OP_DEBUG_PARAMS(maker, line, file, reason, edge->id, oEdge->id));
    return next;
}

void OpContour::addLine(const OpPoint pts[2]) {
    if (pts[0] == pts[1])   // !!! should be fill only, not frame
        return;
    LinePts linePts = {{ pts[0], pts[1] }};
    segments.emplace_back(linePts, this  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
}

OpContour* OpContour::addMove(const OpPoint pts[1]) {
            // !!! if frame paths are supported, don't add close unless fill is set
    if (addClose())
        contours->contours.emplace_back(contours, operand);
    // keep point as its own segment in the future?
    return &contours->contours.back();
}

OpIntersection* OpContour::addSegSect(const OpPtT& t, OpSegment* seg, int cID, int uID
        OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason,
        const OpSegment* oSeg)) {
    OpIntersection* next = contours->allocateIntersection();
    next->set(t, seg, cID, uID  OP_DEBUG_PARAMS(maker, line, file, reason, seg->id, oSeg->id));
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
        OpCurve whole(pts, OpType::quad);
        if (whole.isLinear()) {
            LinePts linePts = {{ whole.pts[0], whole.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else
            segments.emplace_back(whole, OpType::quad, this  
                    OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        return;
    }
    ExtremaT start = {{ pts[0], 0 }  OP_DEBUG_PARAMS(SectReason::startPt)};
    for (unsigned index = 0; index <= extrema.size(); ++index) {
        ExtremaT end = index < extrema.size() ? extrema[index] :
                ExtremaT({ pts[2], 1 }  OP_DEBUG_PARAMS(SectReason::startPt));
        OpCurve partPts = quad.subDivide(start.ptT, end.ptT);
        if (partPts.isLinear()) {
            LinePts linePts = {{ partPts.pts[0], partPts.pts[2] }};
            if (!linePts.isPoint())
                segments.emplace_back(linePts, this  
                        OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
        } else 
            segments.emplace_back(partPts, OpType::quad, this  
                    OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
}


void OpContour::finish() {
// after all segments in contour have been allocated
    if (!segments.size())
        return;
    OP_ASSERT(segments.size() >= 2);
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

// end of contour; start of contours

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
    OpJoiner joiner(*this, path);  // collect active edges and sort them
    if (!joiner.inX.size())  // !!! all remaining edges could be unsectable...
        return true;
    for (auto edge : joiner.inX) {
        edge->setActive(true);
    }
    for (auto unsectable : joiner.unsectInX) {
        unsectable->setActive(true);
    }
    // unsortables are not marked active, since they may or may not be part of the output
#if 01 && OP_DEBUG
    ::clear();
    ::hideSegmentEdges();
    ::hideIntersections();
    joiner.debugDraw();
    ::add(unsortables);
    ::redraw();
    OpDebugOut("");
#endif
    joiner.linkUnambiguous();
#if 01 && OP_DEBUG
    OpDebugOut("");
#endif
    return joiner.linkRemaining();
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
// To do this, insert OP_DEBUG_COUNT(*this, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
// !!! OP_DEBUG_COUNT was unintentionally deleted at some point. Hopefully it is in git history...
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
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        OP_DEBUG_FAIL(*this, false);
    sortIntersections();
    makeEdges();
    windCoincidences();  // for partial h/v lines, compute their winding considiering coincidence
    OpWinder windingEdges(*this, EdgesToSort::byCenter);
    FoundWindings foundWindings = windingEdges.setWindings(this);  // walk edge list, compute windings
    if (FoundWindings::fail == foundWindings)
        OP_DEBUG_FAIL(*this, false);
    apply();  // suppress edges which don't meet op criteria
    if (!assemble(result))
        OP_DEBUG_FAIL(*this, false);
    bool inverseFill = OutInverse[+opOperator][leftIn.isInverted()][rightIn.isInverted()];
    result.setInverted(inverseFill);
    OP_DEBUG_SUCCESS(*this, true);
}

#if OP_DEBUG
void OpContour::debugComplete() {
    id = contours->id++;
}
#endif

