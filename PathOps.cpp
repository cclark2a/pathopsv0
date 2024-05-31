// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

bool PathOps(OpInPath& left, OpInPath& right, OpOperator opOperator, OpOutPath& result) {
    OpContours contourList(left, right, opOperator);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = &contourList;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
    oo();
#endif
    return contourList.pathOps(result);
}

#if OP_DEBUG
// entry point if operation success is already known
bool DebugPathOps(OpInPath& left, OpInPath& right, OpOperator opOperator, OpOutPath& result,
        OpDebugExpect expected, std::string testname, std::vector<OpDebugWarning>& warn) {
    OpContours contourList(left, right, opOperator);
    contourList.debugExpect = expected;
    contourList.debugResult = &result;
    contourList.debugInPathOps = true;
    contourList.debugInClearEdges = false;
    contourList.debugTestname = testname;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = &contourList;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
//    oo();
#endif
    bool success = contourList.pathOps(result);
    warn = contourList.debugWarnings;
    return success;
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
