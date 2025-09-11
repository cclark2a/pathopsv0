// (c) 2024, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/CutWinding.h"

using namespace PathOpsV0Lib;

// curve types
constexpr CurveType line = 1;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr CurveType quad = 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;

WindKeep cutExampleOutput(Curve c, Winding , bool firstPt, bool lastPt) {
    std::string outStr = line == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += pt.toString() + delimiter;
    };
    addPtStr(c.data->start, ", ");
	if (quad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
    return WindKeep::Discard;
}

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void CutExample() {
    CutData cutData(CutDirection::clockwise);
    Context* context = cutContext((ContextUserData*) &cutData, cutExampleOutput);
    lineCallbacks(context, line);
    quadCallbacks(context, quad);
    FrameWinding cutWinding(context, FrameFill::frame);
    FrameWinding fillWinding(context, FrameFill::fill);

	OpPoint linePts[] { { 10, 10 }, { 20, 20 } };
	OpPoint quadPts[] { { 30, 30 }, { 40, 30 }, { 50, 50 } };
    AddLine(cutWinding.winding.contour, { context, linePts, lineSize, line } );
    AddQuads(cutWinding.winding.contour, { context, quadPts, quadSize, quad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };
	for (int index = 0; index < 4; ++index)
		AddLine(fillWinding.winding.contour, { context, &rect[index], lineSize, line } );
	SetErrorHandler(context, allowDisjointLines);
    cutCallbacks(context);
    Resolve(context);
    cutData = CutData(CutDirection::counterclockwise);
    Resolve(context);
    DeleteContext(context);
}

OP_TINY_MAIN(CutExample)  // main() for cmake
