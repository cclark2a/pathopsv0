#include "OpContour.h"
#include "OpSegmentBuilder.h"
#include "OpTightBounds.h"

void OpSegmentBuilder::AddConic(OpContour* contour, const OpPoint pts[3], float weight) {
    OpTightBounds bounds;
    OpConic conic(pts, weight);
    bounds.calcBounds(conic);
    std::vector<ExtremaT> extrema = FindExtrema(bounds);
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }, weight };
        contour->addSegment(whole, OpType::conic  OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
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
        contour->segments.emplace_back(partPts, OpType::conic, contour
                OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
}

void OpSegmentBuilder::AddCubic(OpContour* contour, const OpPoint pts[4]) {
    // reduction to point if pt 0 equals pt 3 complicated since it requires pts 1, 2 be linear..
    assert(pts[0] != pts[3]); // !!! detect possible degenerate to code from actual test data
    OpTightBounds bounds;
    OpCubic cubic(pts);
    bounds.calcBounds(cubic);
    std::vector<ExtremaT> extrema = FindExtrema(bounds);
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2], pts[3] }, 1};
        contour->segments.emplace_back(whole, OpType::cubic, contour
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
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
        contour->segments.emplace_back(partPts, OpType::cubic, contour
                OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
}

void OpSegmentBuilder::AddQuad(OpContour* contour, const OpPoint pts[3]) {
    OpTightBounds bounds;
    OpQuad quad(pts);
    bounds.calcBounds(quad);
    std::vector<ExtremaT> extrema = FindExtrema(bounds);
    if (!extrema.size()) {
        CurvePts whole = {{ pts[0], pts[1], pts[2] }, 1};
        contour->segments.emplace_back(whole, OpType::quad, contour
                OP_DEBUG_PARAMS(SectReason::startPt, SectReason::endPt));
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
        contour->segments.emplace_back(partPts, OpType::quad, contour
                OP_DEBUG_PARAMS(start.reason, end.reason));
        start = end;
    }
}
