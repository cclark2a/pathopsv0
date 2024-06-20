// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

bool PathOps(OpInPath& left, OpInPath& right, OpOperator opOperator, OpOutPath& result
        OP_DEBUG_PARAMS(OpDebugData& debugData)) {
    OpContours contourList(left, right, opOperator);
#if OP_DEBUG
    contourList.debugExpect = debugData.debugExpect;
    contourList.debugResult = &result;
    contourList.debugInPathOps = true;
    contourList.debugInClearEdges = false;
    contourList.debugTestname = debugData.debugTestname;
    contourList.dumpCurve1 = debugData.debugCurveCurve1;
    contourList.dumpCurve2 = debugData.debugCurveCurve2;
    contourList.debugBreakDepth = debugData.debugCurveCurveDepth;
    contourList.newInterface = false;
#endif
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = &contourList;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
//    oo();
#endif
    bool success = contourList.pathOps(result);
#if OP_DEBUG
    debugData.debugWarnings = contourList.debugWarnings;
#endif
    return success;
}

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
