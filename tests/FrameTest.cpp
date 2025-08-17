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

void TestFrame() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext();
    SetContextCallbacks(context, { frameOutput });
    frameCallbacks(context);

#if OP_DEBUG
    OpDebugData debugData(false);
    Debug(context, debugData);
	SetDebugContextCallbacks(context, { // nullptr  
			OP_DEBUG_DUMP_CODE(nullptr, frameDumpOutFunc) });
#endif

    SetCurveCallbacks(context, frameLine, { } );
    quadCallbacks(context, frameQuad);
    FrameWinding frameData(FrameFill::frame, 1);
    Winding frameWinding { &frameData, sizeof(frameData) };
    Contour* frameContour = CreateContour(context, frameWinding);
#if OP_DEBUG
	FrameFill frameContourData = FrameFill::frame;
	SetDebugContourData(frameContour, { &frameContourData, sizeof(frameContourData) }, 
            DebugContourType::windingUserData );
#endif

    FrameWinding fillData(FrameFill::fill, 1);
    Winding fillWinding { &fillData, sizeof(fillData) };
    Contour* fillContour = CreateContour(context, fillWinding);
#if OP_DEBUG
	FrameFill fillContourData = FrameFill::fill;
	SetDebugContourData(fillContour, { &fillContourData, sizeof(fillContourData) }, 
            DebugContourType::windingUserData );
#endif

	// example: return line parts in hourglass fill
#if 0
    OpPoint line[] { { 2, 0 }, { 0, 2 } };
    Add({ line, frameLineSize, frameLine }, frameAddWinding );
    OpPoint hourglass[] { { 0.5, 1 }, { 2.5, 1 }, { 1.5, 0 }, { 1.5, 2 }, { 0.5, 1 } };
	for (int index = 0; index < 4; ++index)
		Add({ &hourglass[index], frameLineSize, frameLine }, fillAddWinding );
#else
	OpPoint line[] { { 10, 10 }, { 20, 20 } };
	OpPoint quad[] { { 30, 30 }, { 50, 50 }, { 40, 30 } };
    Add(frameContour, { context, line, frameLineSize, frameLine } );
    Add(frameContour, { context, quad, frameQuadSize, frameQuad } );
    OpPoint rect[] { { 15, 15 }, { 45, 15 }, { 45, 45 }, { 15, 45 }, { 15, 15 } };
	for (int index = 0; index < 4; ++index)
		Add(fillContour, { context, &rect[index], frameLineSize, frameLine } );
#endif

	SetErrorHandler(context, allowDisjointLines);
    Resolve(context, nullptr);
	ContextError error = Error(context);
    DeleteContext(context);
    if (ContextError::none != error)
        exit(1);
}

OP_TINY_MAIN(TestFrame)  // main() for cmake
