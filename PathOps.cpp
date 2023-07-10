#include "OpContour.h"
#include "PathOps.h"

bool PathOps(OpInPath left, OpInPath right, OpOperator _operator, OpOutPath result) {
    OpContours contourList(left, right, _operator);
    return contourList.pathOps(result);
}

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath left, OpInPath right, OpOperator _operator, OpOutPath result,
        OpDebugExpect expected) {
    OpContours contourList(left, right, _operator);
    contourList.debugExpect = expected;
    contourList.debugResult = &result;
    contourList.debugInPathOps = true;
    contourList.debugInClearEdges = false;
    debugGlobalContours = &contourList;
#if OP_DEBUG_IMAGE
    OpDebugImage::init(left.skPath, right.skPath);
    oo();
#endif
    return contourList.pathOps(result);
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
