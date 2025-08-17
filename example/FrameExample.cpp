// (c) 2024, Cary Clark cclark2@gmail.com

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

void FrameExample() {
    Context* context = CreateContext();
    SetContextCallbacks(context, { frameOutput });
    frameCallbacks(context);

    SetCurveCallbacks(context, frameLine, { } );
    quadCallbacks(context, frameQuad);
    FrameWinding frameData(FrameFill::frame, 1);
    Winding frameWinding { &frameData, sizeof(frameData) };
    Contour* frameContour = CreateContour(context, frameWinding);
    FrameWinding fillData(FrameFill::fill, 1);
    Winding fillWinding { &fillData, sizeof(fillData) };
    Contour* fillContour = CreateContour(context, fillWinding);

	OpPoint line[] { { 10, 10 }, { 20, 20 } };
	OpPoint quad[] { { 30, 30 }, { 50, 50 }, { 40, 30 } };
    Add(frameContour, { context, line, frameLineSize, frameLine } );
    Add(frameContour, { context, quad, frameQuadSize, frameQuad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };

	for (int index = 0; index < 4; ++index)
		Add(fillContour, { context, &rect[index], frameLineSize, frameLine } );
	SetErrorHandler(context, allowDisjointLines);
    Resolve(context, nullptr);
    DeleteContext(context);
}

OP_TINY_MAIN(FrameExample)  // main() for cmake
