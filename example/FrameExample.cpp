// (c) 2024, Cary Clark cclark2@gmail.com

/* !!! add comment here
*/

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/FrameWinding.h"

using namespace PathOpsV0Lib;

// curve types
constexpr CurveType frameLine = 1;
constexpr size_t frameLineSize = sizeof(OpPoint) * 2;
constexpr CurveType frameQuad = 2;
constexpr size_t frameQuadSize = sizeof(OpPoint) * 3;

static WindKeep frameOutput(Output o) {
    const Curve& c = o.curve;
    std::string outStr = frameLine == c.type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += pt.toString() + delimiter;
    };
    addPtStr(c.data->start, ", ");
	if (frameQuad == c.type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
    return WindKeep::Discard;
}

static bool allowDisjointLines(ContextError err, Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

void FrameExample() {
    Context* context = frameContext(frameOutput);
    lineCallbacks(context, frameLine);
    quadCallbacks(context, frameQuad);
    FrameWinding frame(context, FrameFill::frame);
    FrameWinding fill(context, FrameFill::fill);

	OpPoint line[] { { 10, 10 }, { 20, 20 } };
	OpPoint quad[] { { 30, 30 }, { 40, 30 }, { 50, 50 } };
    AddLine(frame.winding.contour, { context, line, frameLineSize, frameLine } );
    AddQuads(frame.winding.contour, { context, quad, frameQuadSize, frameQuad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };

	for (int index = 0; index < 4; ++index)
		AddLine(fill.winding.contour, { context, &rect[index], frameLineSize, frameLine } );
	SetErrorHandler(context, allowDisjointLines);
    Resolve(context);
    DeleteContext(context);
}

OP_TINY_MAIN(FrameExample)  // main() for cmake
