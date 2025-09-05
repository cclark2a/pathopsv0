// (c) 2024, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/FrameWinding.h"

using namespace PathOpsV0Lib;

// curve types
constexpr CurveType frameLine = 1;
constexpr size_t frameLineSize = sizeof(OpPoint) * 2;
constexpr CurveType frameQuad = 2;
constexpr size_t frameQuadSize = sizeof(OpPoint) * 3;

static void frameOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    std::string outStr = frameLine == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");
	if (frameQuad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
}

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void TestFrame() {
    Context* context = frameContext(frameOutput);
    lineCallbacks(context, frameLine );
    quadCallbacks(context, frameQuad);
    FrameWinding frameWinding(context, FrameFill::frame);
    FrameWinding fillWinding(context, FrameFill::fill);
	OpPoint line[] { { 10, 10 }, { 20, 20 } };
	OpPoint quad[] { { 30, 30 }, { 50, 50 }, { 40, 30 } };
    AddLine(frameWinding.contour, { context, line, frameLineSize, frameLine } );
    AddQuads(frameWinding.contour, { context, quad, frameQuadSize, frameQuad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };
	for (int index = 0; index < 4; ++index)
		AddLine(fillWinding.contour, { context, &rect[index], frameLineSize, frameLine } );

	SetErrorHandler(context, allowDisjointLines);
    Resolve(context, nullptr);
	ContextError error = Error(context);
    DeleteContext(context);
    if (ContextError::none != error)
        exit(1);
}

OP_TINY_MAIN(TestFrame)  // main() for cmake
