#include "OpContour.h"
#include "OpEdgeBuilder.h"
#include "OpEdges.h"
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

static const bool OutInverse[+OpOperator::ReverseSubtract + 1][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};

// Early work should ensure that all intersections use segment data only.
// Once edges are constructed, subsequent intersections should use edge data only.
// 
// If successive runs of the same input are flaky, check to see if identical ids are generated.
// To do this, insert OP_DEBUG_COUNT(contourList, _some_identifer_); after every callout.  
// This will compare the dumps of contours and contents to detect when something changed.
// The callouts are removed when not in use as they are not maintained and reduce readability.
bool PathOps(OpInPath left, OpInPath right, OpOperator _operator, OpOutPath result) {
    _operator = OpInverse[+_operator][left.isInverted()][right.isInverted()];
    bool inverseFill = OutInverse[+_operator][left.isInverted()][right.isInverted()];
    OpContours contourList(_operator);
#if OP_DEBUG
    OpDebugPathOpsEnable debugEnable;
    debugGlobalContours = &contourList;
    debugGlobalIntersect = OpDebugIntersect::segment;
#endif
#if 01 && OP_DEBUG_IMAGE
    OpDebugImage::init(left.skPath, right.skPath);
    oo();
#endif
    if (!OpSegmentBuilder::Build(left, contourList, OpOperand::left))
        return false;
    if (!OpSegmentBuilder::Build(right, contourList, OpOperand::right))
        return false;
    contourList.setBounds();
    OpSegments sortedSegments(contourList);
    if (!sortedSegments.inX.size())
        return result.setEmpty();
    sortedSegments.findCoincidences();
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        return false;
#if OP_DEBUG
    debugGlobalIntersect = OpDebugIntersect::edge;
#endif
    contourList.findSegmentExtrema();
    contourList.resolvePoints();    // multiple points may have same t value
    contourList.calcBounds();   // resolve points may have changed tight bounds
    contourList.makeEdges();
    OpEdges sortedEdges(contourList, EdgesToSort::byBox);
    if (!sortedEdges.inX.size())
        return result.setEmpty();
    if (FoundIntersections::fail == sortedEdges.findIntersections())
        return false;
    contourList.sortIntersections();
    contourList.missingCoincidence();  // add intersections for indirect coincidence
    // at this point, edges curves broken at extrema and inflection;
    //   intersections are ptT for each found crossing
    contourList.sortIntersections();    // !!! should do nothing if intersections are unchanged
    contourList.resolvePoints();    // added coincident points may have multiple pts with single t
    contourList.intersectEdge();  // combine edge list and intersection list
    if (!contourList.resolveCoincidence())  // leave at most one active for each pair of coincident edges
        return false;
    OpEdges windingEdges(contourList, EdgesToSort::byCenter);
    FoundWindings foundWindings = windingEdges.setWindings(&contourList);  // walk edge list, compute windings
    if (FoundWindings::fail == foundWindings)
        return false;
    contourList.apply();  // suppress edges which don't meet op criteria
    // A segment may contain multiple intersections with the same t and different points.
    // If found, replace all matching points with the average, in this and the intersected segment.
    // !!! probably not needed if done earlier (if needed, must be rewritten)
    contourList.resolvePoints();
    if (!OpEdgeBuilder::Assemble(contourList, result))
        return false;
    return result.setInverted(inverseFill);
    return true;
}

// if needed, this implementation would reduce the curves' order
bool PathReduce(const OpInPath& path, OpOutPath* result) {
    assert(0);
    return false;
}

// if needed, this implementation remove zero area contours
bool PathCleanUp(const OpInPath& path, OpOutPath* result) {
    assert(0);
    return false;
}
