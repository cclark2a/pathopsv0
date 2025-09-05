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

void cutExampleOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    std::string outStr = line == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");
	if (quad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
}

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void CutExample() {
    Context* context = frameContext(cutExampleOutput);
    lineCallbacks(context, line);
    quadCallbacks(context, quad);
    FrameWinding cutWinding(context, FrameFill::frame);
    FrameWinding fillWinding(context, FrameFill::fill);

	OpPoint linePts[] { { 10, 10 }, { 20, 20 } };
	OpPoint quadPts[] { { 30, 30 }, { 40, 30 }, { 50, 50 } };
    AddLine(cutWinding.contour, { context, linePts, lineSize, line } );
    AddQuads(cutWinding.contour, { context, quadPts, quadSize, quad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };

	for (int index = 0; index < 4; ++index)
		AddLine(fillWinding.contour, { context, &rect[index], lineSize, line } );
	SetErrorHandler(context, allowDisjointLines);
    cutLeftCallbacks(context);
    Resolve(context, nullptr);
    cutRightCallbacks(context);
    Resolve(context, nullptr);
    DeleteContext(context);
}

OP_TINY_MAIN(CutExample)  // main() for cmake
