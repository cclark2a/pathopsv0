#include "OpContour.h"
#include "OpEdgeBuilder.h"
#include "OpEdges.h"
#include "OpSegments.h"
#include "PathOps.h"

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

static bool InnerPathOps(OpContours& contourList, OpInPath left, OpInPath right, 
        OpOperator _operator, OpOutPath result) {
    if (!contourList.build(left, OpOperand::left))  // builds monotonic segments, and adds 0/1 sects
        OP_DEBUG_FAIL(contourList, false);
    if (!contourList.build(right, OpOperand::right))
        OP_DEBUG_FAIL(contourList, false);
    contourList.finishAll();
    contourList.setBounds();    // !!! check to see if this is used
    OpSegments sortedSegments(contourList);
    if (!sortedSegments.inX.size()) {
        result.setEmpty();
        OP_DEBUG_SUCCESS(contourList, true);
    }
    sortedSegments.findCoincidences();  // check for exact curves and full lines
    sortedSegments.findLineCoincidences();  // check for partial h/v lines
    if (FoundIntersections::fail == sortedSegments.findIntersections())
        OP_DEBUG_FAIL(contourList, false);
//    contourList.resolvePoints();    // multiple points may have same t value
//    contourList.calcBounds();   // resolve points may have changed tight bounds
    contourList.sortIntersections();
    contourList.makeEdges();
//    OpEdges sortedEdges(contourList, EdgesToSort::byBox);
//    if (!sortedEdges.inX.size()) {
//        result.setEmpty();
//        OP_DEBUG_SUCCESS(contourList, true);
//    }
//    if (FoundIntersections::fail == sortedEdges.findIntersections())
//        OP_DEBUG_FAIL(contourList, false);
//    contourList.missingCoincidence();  // add intersections for indirect coincidence
    // at this point, edges curves broken at extrema and inflection;
    //   intersections are ptT for each found crossing
 //   contourList.sortIntersections();    // !!! should do nothing if intersections are unchanged
//    contourList.resolvePoints();    // added coincident points may have multiple pts with single t
//    contourList.intersectEdge();  // combine edge list and intersection list
//    if (!contourList.resolveCoincidence())  // leave at most one active for each pair of coincident edges
//        OP_DEBUG_FAIL(contourList, false);
    OpEdges windingEdges(contourList, EdgesToSort::byCenter);
    FoundWindings foundWindings = windingEdges.setWindings(&contourList);  // walk edge list, compute windings
    if (FoundWindings::fail == foundWindings)
        OP_DEBUG_FAIL(contourList, false);
    contourList.apply();  // suppress edges which don't meet op criteria
    // A segment may contain multiple intersections with the same t and different points.
    // If found, replace all matching points with the average, in this and the intersected segment.
    // !!! probably not needed if done earlier (if needed, must be rewritten)
//    contourList.resolvePoints();
    if (!OpEdgeBuilder::Assemble(contourList, result))
        OP_DEBUG_FAIL(contourList, false);
    bool inverseFill = OutInverse[+contourList._operator][left.isInverted()][right.isInverted()];
    result.setInverted(inverseFill);
    OP_DEBUG_SUCCESS(contourList, true);
}

bool PathOps(OpInPath left, OpInPath right, OpOperator _operator, OpOutPath result) {
    OpContours contourList(left, right, _operator);
    return InnerPathOps(contourList, left, right, _operator, result);
}

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath left, OpInPath right, OpOperator _operator, OpOutPath result,
        OpDebugExpect expected) {
    OpContours contourList(left, right, _operator);
    contourList.debugExpect = expected;
    contourList.debugInPathOps = true;
    contourList.debugInClearEdges = false;
    debugGlobalContours = &contourList;
#if OP_DEBUG_IMAGE
    OpDebugImage::init(left.skPath, right.skPath);
    oo();
#endif
    return InnerPathOps(contourList, left, right, _operator, result);
}
#endif

// if needed, this implementation would reduce the curves' order
bool PathReduce(const OpInPath& path, OpOutPath* result) {
    OP_ASSERT(0);
    return false;
}

// if needed, this implementation remove zero area contours
bool PathCleanUp(const OpInPath& path, OpOutPath* result) {
    OP_ASSERT(0);
    return false;
}
